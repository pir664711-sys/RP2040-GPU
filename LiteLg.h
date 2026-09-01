// LiteLg.h
// -----------------------------------------------------------------------
// Handle-based Windows client for the PicoDVI 2D-accel serial protocol.
// Wire format is unchanged: [0xAA][opcode][payload...], big-endian
// multi-byte fields, single-byte 0x06 ACK per packet (0xAA for ping).
//
// API shape: LiteLg_Open() returns an opaque device handle (or NULL on
// failure). Every other call takes that handle as its first argument
// and returns one of the LITELG_* result codes below, so you can run
// more than one board/port from the same process if you ever need to.
// -----------------------------------------------------------------------
#ifndef LITELG_H
#define LITELG_H

#include <stdint.h>

#if defined(LITELG_BUILD_DLL)
#define LITELG_API __declspec(dllexport)
#elif defined(LITELG_STATIC)
#define LITELG_API
#else
#define LITELG_API __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

    enum LiteLgResult {
        LITELG_OK = 0,
        LITELG_ERR_PARAM = -1, // null handle / bad argument
        LITELG_ERR_WRITE = -2, // WriteFile to the COM port failed
        LITELG_ERR_TIMEOUT = -3, // board did not reply within the timeout
        LITELG_ERR_NACK = -4  // board replied, but not with the expected byte
    };

    // Wire-protocol opcodes -- must stay identical to the Pico firmware.
    enum LiteLgOpcode {
        LITELG_CMD_CLEAR = 0x01,
        LITELG_CMD_SET_PALETTE = 0x02,
        LITELG_CMD_PIXEL = 0x03,
        LITELG_CMD_LINE = 0x04,
        LITELG_CMD_RECT = 0x05,
        LITELG_CMD_FILL_RECT = 0x06,
        LITELG_CMD_CIRCLE = 0x07,
        LITELG_CMD_FILL_CIRCLE = 0x08,
        LITELG_CMD_TRIANGLE = 0x09,
        LITELG_CMD_FILL_TRIANGLE = 0x0A,
        LITELG_CMD_SWAP = 0x0B,
        LITELG_CMD_PING = 0xF0
    };

    // ---- Discovery & connection ---------------------------------------------

    // Scans for COM ports that currently exist on the system and writes a
    // human-readable, comma-separated list (e.g. "COM3, COM8") into buf.
    // Returns the number of ports found (0 if none, buf left as an empty
    // string in that case).
    LITELG_API int LiteLg_ListPorts(char* buf, int bufLen);

    // Opens portName (e.g. "COM8") at the given baud rate. Returns an
    // opaque device handle to pass to every other call, or NULL on
    // failure (bad port name, already in use, etc). Every successful call
    // must eventually be matched with LiteLg_Close.
    LITELG_API void* LiteLg_Open(const char* portName, int baud);

    // Closes the port and frees the handle. Safe to call with NULL.
    LITELG_API void  LiteLg_Close(void* dev);

    // Sends CMD_PING and waits for the board's 0xAA reply (not the generic
    // 0x06 ACK). Returns LITELG_OK if the board answered in time.
    LITELG_API int LiteLg_Ping(void* dev);

    // ---- Drawing primitives -------------------------------------------------
    // Every call returns a LiteLgResult. Coordinates/sizes go over the wire
    // as big-endian int16; color/index/component values are single bytes.

    LITELG_API int LiteLg_Clear(void* dev, uint8_t color);
    LITELG_API int LiteLg_SetPalette(void* dev, uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    LITELG_API int LiteLg_Pixel(void* dev, int16_t x, int16_t y, uint8_t color);
    LITELG_API int LiteLg_Line(void* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);
    LITELG_API int LiteLg_Rect(void* dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
    LITELG_API int LiteLg_FillRect(void* dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
    LITELG_API int LiteLg_Circle(void* dev, int16_t x, int16_t y, int16_t r, uint8_t color);
    LITELG_API int LiteLg_FillCircle(void* dev, int16_t x, int16_t y, int16_t r, uint8_t color);
    LITELG_API int LiteLg_Triangle(void* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color);
    LITELG_API int LiteLg_FillTriangle(void* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color);

    // Presents the back buffer (CMD_SWAP, copy=true on the firmware side).
    LITELG_API int LiteLg_Swap(void* dev);

#ifdef __cplusplus
}
#endif

#endif // LITELG_H