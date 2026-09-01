// LiteLg.cpp
// -----------------------------------------------------------------------
// Implementation of the handle-based LiteLg.dll. Each LiteLg_Open call
// creates its own LiteLgDevice with its own COM handle, its own critical
// section, and its own last-error text -- so multiple devices are fully
// independent and safe to use from multiple threads.
// -----------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS
#define LITELG_BUILD_DLL
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdlib>
#include "LiteLg.h"

namespace {

    const uint8_t PKT_START = 0xAA;
    const uint8_t PKT_ACK = 0x06;
    const uint8_t PING_REPLY = 0xAA;

    // Firmware does readExact() with a 1000ms per-byte timeout before giving
    // up on a packet, so give the ACK a bit more slack for USB-serial latency.
    const DWORD ACK_TIMEOUT_MS = 1500;

    struct LiteLgDevice {
        HANDLE            hPort;
        CRITICAL_SECTION  lock;
    };

    void PushI16(uint8_t* buf, int& pos, int16_t v) {
        buf[pos++] = (uint8_t)((v >> 8) & 0xFF);
        buf[pos++] = (uint8_t)(v & 0xFF);
    }

    bool LooksLikeComPort(const char* s, size_t len) {
        if (len < 4) return false; // "COM" + at least one digit
        if (toupper((unsigned char)s[0]) != 'C' ||
            toupper((unsigned char)s[1]) != 'O' ||
            toupper((unsigned char)s[2]) != 'M') return false;
        for (size_t i = 3; i < len; i++) {
            if (!isdigit((unsigned char)s[i])) return false;
        }
        return true;
    }

    // Sends [0xAA][opcode][payload...] on dev->hPort and waits for a single
    // reply byte matching expectedReply (0x06 for normal commands, 0xAA for
    // ping). Caller must hold dev->lock.
    int SendPacketLocked(LiteLgDevice* dev, uint8_t opcode,
        const uint8_t* payload, int payloadLen,
        uint8_t expectedReply) {
        uint8_t buf[2 + 32];
        int pos = 0;
        buf[pos++] = PKT_START;
        buf[pos++] = opcode;
        if (payloadLen > 0) {
            memcpy(buf + pos, payload, payloadLen);
            pos += payloadLen;
        }

        DWORD written = 0;
        if (!WriteFile(dev->hPort, buf, (DWORD)pos, &written, nullptr) || written != (DWORD)pos) {
            return LITELG_ERR_WRITE;
        }

        uint8_t reply = 0;
        DWORD readBytes = 0;
        if (!ReadFile(dev->hPort, &reply, 1, &readBytes, nullptr) || readBytes != 1) {
            return LITELG_ERR_TIMEOUT;
        }
        if (reply != expectedReply) {
            return LITELG_ERR_NACK;
        }
        return LITELG_OK;
    }

    int SendCommand(void* devHandle, uint8_t opcode, const uint8_t* payload, int payloadLen) {
        LiteLgDevice* dev = (LiteLgDevice*)devHandle;
        if (!dev || dev->hPort == INVALID_HANDLE_VALUE) return LITELG_ERR_PARAM;
        EnterCriticalSection(&dev->lock);
        int r = SendPacketLocked(dev, opcode, payload, payloadLen, PKT_ACK);
        LeaveCriticalSection(&dev->lock);
        return r;
    }

} // namespace

// ---------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------

int LiteLg_ListPorts(char* buf, int bufLen) {
    if (buf && bufLen > 0) buf[0] = '\0';

    char dosBuf[8192];
    DWORD len = QueryDosDeviceA(nullptr, dosBuf, sizeof(dosBuf));
    if (len == 0) return 0;

    int count = 0;
    char* p = dosBuf;
    while (*p) {
        size_t plen = strlen(p);
        if (LooksLikeComPort(p, plen)) {
            if (buf && bufLen > 0) {
                if (count > 0) strncat(buf, ", ", (size_t)bufLen - strlen(buf) - 1);
                strncat(buf, p, (size_t)bufLen - strlen(buf) - 1);
            }
            count++;
        }
        p += plen + 1;
    }
    return count;
}

// ---------------------------------------------------------------------
// Connection management
// ---------------------------------------------------------------------

void* LiteLg_Open(const char* portName, int baud) {
    if (!portName || !*portName) return nullptr;

    char fullName[64];
    _snprintf(fullName, sizeof(fullName) - 1, "\\\\.\\%s", portName);
    fullName[sizeof(fullName) - 1] = '\0';

    HANDLE h = CreateFileA(fullName, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return nullptr;

    DCB dcb;
    ZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) { CloseHandle(h); return nullptr; }
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE; // many Pico boards reset on DTR
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); return nullptr; }

    COMMTIMEOUTS timeouts;
    ZeroMemory(&timeouts, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = ACK_TIMEOUT_MS;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(h, &timeouts);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    // RP2040 boards commonly reboot when the port opens (DTR toggling);
    // give the sketch's setup() (palette init, framebuffer clear) time
    // to finish before we start sending draw commands.
    Sleep(1500);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    LiteLgDevice* dev = (LiteLgDevice*)malloc(sizeof(LiteLgDevice));
    if (!dev) { CloseHandle(h); return nullptr; }
    dev->hPort = h;
    InitializeCriticalSection(&dev->lock);
    return (void*)dev;
}

void LiteLg_Close(void* devHandle) {
    LiteLgDevice* dev = (LiteLgDevice*)devHandle;
    if (!dev) return;
    if (dev->hPort != INVALID_HANDLE_VALUE) {
        CloseHandle(dev->hPort);
        dev->hPort = INVALID_HANDLE_VALUE;
    }
    DeleteCriticalSection(&dev->lock);
    free(dev);
}

int LiteLg_Ping(void* devHandle) {
    LiteLgDevice* dev = (LiteLgDevice*)devHandle;
    if (!dev || dev->hPort == INVALID_HANDLE_VALUE) return LITELG_ERR_PARAM;
    EnterCriticalSection(&dev->lock);
    int r = SendPacketLocked(dev, LITELG_CMD_PING, nullptr, 0, PING_REPLY);
    LeaveCriticalSection(&dev->lock);
    return r;
}

// ---------------------------------------------------------------------
// Drawing primitives
// ---------------------------------------------------------------------

int LiteLg_Clear(void* dev, uint8_t color) {
    uint8_t p[1] = { color };
    return SendCommand(dev, LITELG_CMD_CLEAR, p, 1);
}

int LiteLg_SetPalette(void* dev, uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t p[4] = { index, r, g, b };
    return SendCommand(dev, LITELG_CMD_SET_PALETTE, p, 4);
}

int LiteLg_Pixel(void* dev, int16_t x, int16_t y, uint8_t color) {
    uint8_t p[5]; int pos = 0;
    PushI16(p, pos, x); PushI16(p, pos, y); p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_PIXEL, p, pos);
}

int LiteLg_Line(void* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color) {
    uint8_t p[9]; int pos = 0;
    PushI16(p, pos, x0); PushI16(p, pos, y0);
    PushI16(p, pos, x1); PushI16(p, pos, y1);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_LINE, p, pos);
}

int LiteLg_Rect(void* dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    uint8_t p[9]; int pos = 0;
    PushI16(p, pos, x); PushI16(p, pos, y);
    PushI16(p, pos, w); PushI16(p, pos, h);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_RECT, p, pos);
}

int LiteLg_FillRect(void* dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    uint8_t p[9]; int pos = 0;
    PushI16(p, pos, x); PushI16(p, pos, y);
    PushI16(p, pos, w); PushI16(p, pos, h);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_FILL_RECT, p, pos);
}

int LiteLg_Circle(void* dev, int16_t x, int16_t y, int16_t r, uint8_t color) {
    uint8_t p[7]; int pos = 0;
    PushI16(p, pos, x); PushI16(p, pos, y); PushI16(p, pos, r);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_CIRCLE, p, pos);
}

int LiteLg_FillCircle(void* dev, int16_t x, int16_t y, int16_t r, uint8_t color) {
    uint8_t p[7]; int pos = 0;
    PushI16(p, pos, x); PushI16(p, pos, y); PushI16(p, pos, r);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_FILL_CIRCLE, p, pos);
}

int LiteLg_Triangle(void* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1,
    int16_t x2, int16_t y2, uint8_t color) {
    uint8_t p[13]; int pos = 0;
    PushI16(p, pos, x0); PushI16(p, pos, y0);
    PushI16(p, pos, x1); PushI16(p, pos, y1);
    PushI16(p, pos, x2); PushI16(p, pos, y2);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_TRIANGLE, p, pos);
}

int LiteLg_FillTriangle(void* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1,
    int16_t x2, int16_t y2, uint8_t color) {
    uint8_t p[13]; int pos = 0;
    PushI16(p, pos, x0); PushI16(p, pos, y0);
    PushI16(p, pos, x1); PushI16(p, pos, y1);
    PushI16(p, pos, x2); PushI16(p, pos, y2);
    p[pos++] = color;
    return SendCommand(dev, LITELG_CMD_FILL_TRIANGLE, p, pos);
}

int LiteLg_Swap(void* dev) {
    return SendCommand(dev, LITELG_CMD_SWAP, nullptr, 0);
}

// ---------------------------------------------------------------------
BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID) {
    return TRUE;
}
