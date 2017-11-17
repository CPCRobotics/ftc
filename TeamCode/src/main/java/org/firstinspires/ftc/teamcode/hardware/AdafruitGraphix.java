package org.firstinspires.ftc.teamcode.hardware;

//
// This is a slightly modified and translated form of AdaFruit Graphix library
//
// AdaFruit copyright:
//
// Copyright (c) 2013 Adafruit Industries.  All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

/**
 * Based off Adafruit_GFX Arduino library.
 */

public abstract class AdafruitGraphix {

    public final int width;
    public final int height;
    private int cursorX;
    private int cursorY;
    private short colorFG;
    private short colorBG;
    private int textSize;
    private boolean wrap;

    public final static short BLACK = 0;
    public final static short WHITE = -1;
    public final static short RED = (short)0xf800;
    public final static short GREEN = 0x07e0;
    public final static short BLUE = 0x001f;
    public final static short YELLOW = RED|GREEN;
    public final static short CYAN = BLUE|GREEN;
    public final static short MAGENTA = RED|BLUE;

    protected AdafruitGraphix(int w, int h) {
        this.width = w;
        this.height = h;
        this.cursorX = 0;
        this.cursorY = 0;
        this.colorFG = WHITE;
        this.colorBG = BLACK;
        this.textSize = 1;
        this.wrap = true;
    }

    // Minimal implementation required
    public abstract void drawPixel(int x, int y, short color);

    // Minimal implementation required
    public abstract void display();

    public void drawFastVLine(int x, int y, int h, short color) {
        drawLine(x, y, x, y+h-1, color);
    }

    public void drawFastHLine(int x, int y, int w, short color) {
        drawLine(x, y, x+y-1, y, color);
    }

    public void drawLine(int x0, int y0, int x1, int y1, short color) {
        if(x0 == x1){
            if(y0 > y1) {
                drawFastVLine(x0, y1, y0 - y1 + 1, color);
            } else {
                drawFastVLine(x0, y0, y1 - y0 + 1, color);
            }
        } else if(y0 == y1){
            if(x0 > x1) {
                drawFastHLine(x1, y0, x0 - x1 + 1, color);
            } else {
                drawFastHLine(x0, y0, x1 - x0 + 1, color);
            }
        } else {
            drawLine(x0, y0, x1, y1, color);
        }
    }

    public void drawRect(int x, int y, int w, int h, short color) {
        drawFastHLine(x, y, w, color);
        drawFastHLine(x, y+h-1, w, color);
        drawFastVLine(x, y, h, color);
        drawFastVLine(x+w-1, y, h, color);
    }

    // Draw a rounded rectangle
    public void drawRoundRect(int x, int y, int w, int h, int r, short color) {
        drawFastHLine(x+r  , y    , w-2*r, color); // Top
        drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
        drawFastVLine(x    , y+r  , h-2*r, color); // Left
        drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
        // draw four corners
        drawCircleHelper(x+r    , y+r    , r, 1, color);
        drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
        drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
        drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
    }

    public void fillRoundRect(int x, int y, int w, int h, int r, short color) {
        fillRect(x+r, y, w-2*r, h, color);

        // draw four corners
        fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
        fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
    }

    public void drawCircle(int x0, int y0, int r, short color) {
        int f = 1 - r;
        int ddF_x = 1;
        int ddF_y = -2 * r;
        int x = 0;
        int y = r;

        drawPixel(x0  , y0+r, color);
        drawPixel(x0  , y0-r, color);
        drawPixel(x0+r, y0  , color);
        drawPixel(x0-r, y0  , color);

        while (x<y) {
            if (f >= 0) {
                y--;
                ddF_y += 2;
                f += ddF_y;
            }
            x++;
            ddF_x += 2;
            f += ddF_x;

            drawPixel(x0 + x, y0 + y, color);
            drawPixel(x0 - x, y0 + y, color);
            drawPixel(x0 + x, y0 - y, color);
            drawPixel(x0 - x, y0 - y, color);
            drawPixel(x0 + y, y0 + x, color);
            drawPixel(x0 - y, y0 + x, color);
            drawPixel(x0 + y, y0 - x, color);
            drawPixel(x0 - y, y0 - x, color);
        }
    }

    protected void drawCircleHelper( int x0, int y0, int r, int corner, short color) {
        int f     = 1 - r;
        int ddF_x = 1;
        int ddF_y = -2 * r;
        int x     = 0;
        int y     = r;

        while (x<y) {
            if (f >= 0) {
                y--;
                ddF_y += 2;
                f     += ddF_y;
            }
            x++;
            ddF_x += 2;
            f     += ddF_x;
            if ((corner & 0x4) != 0) {
                drawPixel(x0 + x, y0 + y, color);
                drawPixel(x0 + y, y0 + x, color);
            }
            if ((corner  & 0x2) != 0) {
                drawPixel(x0 + x, y0 - y, color);
                drawPixel(x0 + y, y0 - x, color);
            }
            if ((corner & 0x8) != 0) {
                drawPixel(x0 - y, y0 + x, color);
                drawPixel(x0 - x, y0 + y, color);
            }
            if ((corner & 0x1) != 0) {
                drawPixel(x0 - y, y0 - x, color);
                drawPixel(x0 - x, y0 - y, color);
            }
        }
    }

    public void fillRect(int x, int y, int w, int h, short color) {
        for (int yy=y; yy < y+h; yy++) {
            drawFastHLine(x, yy, w, color);
        }
    }

    public void fillScreen(short color) {
        fillRect(0, 0, width, height, color);
    }

    public void clearScreen() {
        fillScreen(colorBG);
    }

    public void fillCircle(int x0, int y0, int r, short color) {
        drawFastVLine(x0, y0 - r, 2 * r + 1, color);
        fillCircleHelper(x0, y0, r, 3, 0, color);
    }

    protected void fillCircleHelper(int x0, int y0, int r, int corner, int delta, short color) {
        int f     = 1 - r;
        int ddF_x = 1;
        int ddF_y = -2 * r;
        int x     = 0;
        int y     = r;

        while (x<y) {
            if (f >= 0) {
                y--;
                ddF_y += 2;
                f     += ddF_y;
            }
            x++;
            ddF_x += 2;
            f     += ddF_x;

            if ((corner & 0x1) != 0) {
                drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
                drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
            }
            if ((corner & 0x2) != 0) {
                drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
                drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
            }
        }

    }

    public void drawBitmap(int x, int y, byte [] bitmap, int w, int h) {
        int byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
        byte b = 0;

        for(int j=0; j<h; j++, y++) {
            for(int i=0; i<w; i++) {
                if ((i & 7) != 0) {
                    b <<= 1;
                } else {
                    b = bitmap[j * byteWidth + i / 8];
                }
                drawPixel(x+i, y, (b & 0x80) != 0 ? colorFG : colorBG);
            }
        }
    }

    /**
     * Bitmap is full color (5/6/5)
     */
    public void drawRgbBitmap(int x, int y, short [] bitmap, int w, int h) {
        for(int j=0; j<h; j++, y++) {
            for(int i=0; i<w; i++) {
                drawPixel(x+i, y, bitmap[j*w + i]);
            }
        }
    }

    /**
     * Bitmap is full color (5/6/5), one of those colors is identified as transparent
     */
    public void drawRgbBitmap(int x, int y, short [] bitmap, int w, int h, short transColor) {
        for(int j=0; j<h; j++, y++) {
            for(int i=0; i<w; i++) {
                short c = bitmap[j*w + i];
                if (transColor != c) {
                    drawPixel(x + i, y, bitmap[j * w + i]);
                }
            }
        }
    }

    public void drawChar(int x, int y, char c) {
        if ((x >= width) || (y >= height) || (x + 6*textSize <= 0) || (y + 8*textSize <= 0)) {
            return;
        }
        int cc = ((int)c)&0xff;
        for (int i = 0; i < 5; i++) {
            byte line = font[cc+i];
            for (int j = 0; j < 8; j++, line >>= 1) {
                short color = (line & 1) != 0 ? colorFG : colorBG;
                if (textSize == 1) {
                    drawPixel(x+i, y+j, color);
                } else {
                    fillRect(x+i*textSize, y+j*textSize, textSize, textSize, color);
                }
            }
        }
        if (textSize == 1) {
            drawFastVLine(x+5, y, 8, colorBG);
        } else {
            fillRect(x+5*textSize, y, textSize, 8*textSize, colorBG);
        }
    }

    public void write(char c) {
        if (c == '\n') {
            cursorX = 0;
            cursorX += textSize*8;
        } else if (c != '\r') {
            if (wrap && ((cursorX + textSize * 6) > width)) {
                cursorX = 0;
                cursorY += textSize*8;
            }
            drawChar(cursorX, cursorY, c);
            cursorX += textSize*6;
        }
    }

    public void write(String text) {
        for(char c: text.toCharArray()) {
            write(c);
        }
    }

    public void printf(String format, Object ... params) {
        write(String.format(format, params));
    }

    // See AdaFruit glcdfont.c
    // This font is copied as is, translated to Java
    private static final byte font[] = {
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00,
        (byte)0x3E, (byte)0x5B, (byte)0x4F, (byte)0x5B, (byte)0x3E,
        (byte)0x3E, (byte)0x6B, (byte)0x4F, (byte)0x6B, (byte)0x3E,
        (byte)0x1C, (byte)0x3E, (byte)0x7C, (byte)0x3E, (byte)0x1C,
        (byte)0x18, (byte)0x3C, (byte)0x7E, (byte)0x3C, (byte)0x18,
        (byte)0x1C, (byte)0x57, (byte)0x7D, (byte)0x57, (byte)0x1C,
        (byte)0x1C, (byte)0x5E, (byte)0x7F, (byte)0x5E, (byte)0x1C,
        (byte)0x00, (byte)0x18, (byte)0x3C, (byte)0x18, (byte)0x00,
        (byte)0xFF, (byte)0xE7, (byte)0xC3, (byte)0xE7, (byte)0xFF,
        (byte)0x00, (byte)0x18, (byte)0x24, (byte)0x18, (byte)0x00,
        (byte)0xFF, (byte)0xE7, (byte)0xDB, (byte)0xE7, (byte)0xFF,
        (byte)0x30, (byte)0x48, (byte)0x3A, (byte)0x06, (byte)0x0E,
        (byte)0x26, (byte)0x29, (byte)0x79, (byte)0x29, (byte)0x26,
        (byte)0x40, (byte)0x7F, (byte)0x05, (byte)0x05, (byte)0x07,
        (byte)0x40, (byte)0x7F, (byte)0x05, (byte)0x25, (byte)0x3F,
        (byte)0x5A, (byte)0x3C, (byte)0xE7, (byte)0x3C, (byte)0x5A,
        (byte)0x7F, (byte)0x3E, (byte)0x1C, (byte)0x1C, (byte)0x08,
        (byte)0x08, (byte)0x1C, (byte)0x1C, (byte)0x3E, (byte)0x7F,
        (byte)0x14, (byte)0x22, (byte)0x7F, (byte)0x22, (byte)0x14,
        (byte)0x5F, (byte)0x5F, (byte)0x00, (byte)0x5F, (byte)0x5F,
        (byte)0x06, (byte)0x09, (byte)0x7F, (byte)0x01, (byte)0x7F,
        (byte)0x00, (byte)0x66, (byte)0x89, (byte)0x95, (byte)0x6A,
        (byte)0x60, (byte)0x60, (byte)0x60, (byte)0x60, (byte)0x60,
        (byte)0x94, (byte)0xA2, (byte)0xFF, (byte)0xA2, (byte)0x94,
        (byte)0x08, (byte)0x04, (byte)0x7E, (byte)0x04, (byte)0x08,
        (byte)0x10, (byte)0x20, (byte)0x7E, (byte)0x20, (byte)0x10,
        (byte)0x08, (byte)0x08, (byte)0x2A, (byte)0x1C, (byte)0x08,
        (byte)0x08, (byte)0x1C, (byte)0x2A, (byte)0x08, (byte)0x08,
        (byte)0x1E, (byte)0x10, (byte)0x10, (byte)0x10, (byte)0x10,
        (byte)0x0C, (byte)0x1E, (byte)0x0C, (byte)0x1E, (byte)0x0C,
        (byte)0x30, (byte)0x38, (byte)0x3E, (byte)0x38, (byte)0x30,
        (byte)0x06, (byte)0x0E, (byte)0x3E, (byte)0x0E, (byte)0x06,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00,
        (byte)0x00, (byte)0x00, (byte)0x5F, (byte)0x00, (byte)0x00,
        (byte)0x00, (byte)0x07, (byte)0x00, (byte)0x07, (byte)0x00,
        (byte)0x14, (byte)0x7F, (byte)0x14, (byte)0x7F, (byte)0x14,
        (byte)0x24, (byte)0x2A, (byte)0x7F, (byte)0x2A, (byte)0x12,
        (byte)0x23, (byte)0x13, (byte)0x08, (byte)0x64, (byte)0x62,
        (byte)0x36, (byte)0x49, (byte)0x56, (byte)0x20, (byte)0x50,
        (byte)0x00, (byte)0x08, (byte)0x07, (byte)0x03, (byte)0x00,
        (byte)0x00, (byte)0x1C, (byte)0x22, (byte)0x41, (byte)0x00,
        (byte)0x00, (byte)0x41, (byte)0x22, (byte)0x1C, (byte)0x00,
        (byte)0x2A, (byte)0x1C, (byte)0x7F, (byte)0x1C, (byte)0x2A,
        (byte)0x08, (byte)0x08, (byte)0x3E, (byte)0x08, (byte)0x08,
        (byte)0x00, (byte)0x80, (byte)0x70, (byte)0x30, (byte)0x00,
        (byte)0x08, (byte)0x08, (byte)0x08, (byte)0x08, (byte)0x08,
        (byte)0x00, (byte)0x00, (byte)0x60, (byte)0x60, (byte)0x00,
        (byte)0x20, (byte)0x10, (byte)0x08, (byte)0x04, (byte)0x02,
        (byte)0x3E, (byte)0x51, (byte)0x49, (byte)0x45, (byte)0x3E,
        (byte)0x00, (byte)0x42, (byte)0x7F, (byte)0x40, (byte)0x00,
        (byte)0x72, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x46,
        (byte)0x21, (byte)0x41, (byte)0x49, (byte)0x4D, (byte)0x33,
        (byte)0x18, (byte)0x14, (byte)0x12, (byte)0x7F, (byte)0x10,
        (byte)0x27, (byte)0x45, (byte)0x45, (byte)0x45, (byte)0x39,
        (byte)0x3C, (byte)0x4A, (byte)0x49, (byte)0x49, (byte)0x31,
        (byte)0x41, (byte)0x21, (byte)0x11, (byte)0x09, (byte)0x07,
        (byte)0x36, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x36,
        (byte)0x46, (byte)0x49, (byte)0x49, (byte)0x29, (byte)0x1E,
        (byte)0x00, (byte)0x00, (byte)0x14, (byte)0x00, (byte)0x00,
        (byte)0x00, (byte)0x40, (byte)0x34, (byte)0x00, (byte)0x00,
        (byte)0x00, (byte)0x08, (byte)0x14, (byte)0x22, (byte)0x41,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0x14, (byte)0x14,
        (byte)0x00, (byte)0x41, (byte)0x22, (byte)0x14, (byte)0x08,
        (byte)0x02, (byte)0x01, (byte)0x59, (byte)0x09, (byte)0x06,
        (byte)0x3E, (byte)0x41, (byte)0x5D, (byte)0x59, (byte)0x4E,
        (byte)0x7C, (byte)0x12, (byte)0x11, (byte)0x12, (byte)0x7C,
        (byte)0x7F, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x36,
        (byte)0x3E, (byte)0x41, (byte)0x41, (byte)0x41, (byte)0x22,
        (byte)0x7F, (byte)0x41, (byte)0x41, (byte)0x41, (byte)0x3E,
        (byte)0x7F, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x41,
        (byte)0x7F, (byte)0x09, (byte)0x09, (byte)0x09, (byte)0x01,
        (byte)0x3E, (byte)0x41, (byte)0x41, (byte)0x51, (byte)0x73,
        (byte)0x7F, (byte)0x08, (byte)0x08, (byte)0x08, (byte)0x7F,
        (byte)0x00, (byte)0x41, (byte)0x7F, (byte)0x41, (byte)0x00,
        (byte)0x20, (byte)0x40, (byte)0x41, (byte)0x3F, (byte)0x01,
        (byte)0x7F, (byte)0x08, (byte)0x14, (byte)0x22, (byte)0x41,
        (byte)0x7F, (byte)0x40, (byte)0x40, (byte)0x40, (byte)0x40,
        (byte)0x7F, (byte)0x02, (byte)0x1C, (byte)0x02, (byte)0x7F,
        (byte)0x7F, (byte)0x04, (byte)0x08, (byte)0x10, (byte)0x7F,
        (byte)0x3E, (byte)0x41, (byte)0x41, (byte)0x41, (byte)0x3E,
        (byte)0x7F, (byte)0x09, (byte)0x09, (byte)0x09, (byte)0x06,
        (byte)0x3E, (byte)0x41, (byte)0x51, (byte)0x21, (byte)0x5E,
        (byte)0x7F, (byte)0x09, (byte)0x19, (byte)0x29, (byte)0x46,
        (byte)0x26, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x32,
        (byte)0x03, (byte)0x01, (byte)0x7F, (byte)0x01, (byte)0x03,
        (byte)0x3F, (byte)0x40, (byte)0x40, (byte)0x40, (byte)0x3F,
        (byte)0x1F, (byte)0x20, (byte)0x40, (byte)0x20, (byte)0x1F,
        (byte)0x3F, (byte)0x40, (byte)0x38, (byte)0x40, (byte)0x3F,
        (byte)0x63, (byte)0x14, (byte)0x08, (byte)0x14, (byte)0x63,
        (byte)0x03, (byte)0x04, (byte)0x78, (byte)0x04, (byte)0x03,
        (byte)0x61, (byte)0x59, (byte)0x49, (byte)0x4D, (byte)0x43,
        (byte)0x00, (byte)0x7F, (byte)0x41, (byte)0x41, (byte)0x41,
        (byte)0x02, (byte)0x04, (byte)0x08, (byte)0x10, (byte)0x20,
        (byte)0x00, (byte)0x41, (byte)0x41, (byte)0x41, (byte)0x7F,
        (byte)0x04, (byte)0x02, (byte)0x01, (byte)0x02, (byte)0x04,
        (byte)0x40, (byte)0x40, (byte)0x40, (byte)0x40, (byte)0x40,
        (byte)0x00, (byte)0x03, (byte)0x07, (byte)0x08, (byte)0x00,
        (byte)0x20, (byte)0x54, (byte)0x54, (byte)0x78, (byte)0x40,
        (byte)0x7F, (byte)0x28, (byte)0x44, (byte)0x44, (byte)0x38,
        (byte)0x38, (byte)0x44, (byte)0x44, (byte)0x44, (byte)0x28,
        (byte)0x38, (byte)0x44, (byte)0x44, (byte)0x28, (byte)0x7F,
        (byte)0x38, (byte)0x54, (byte)0x54, (byte)0x54, (byte)0x18,
        (byte)0x00, (byte)0x08, (byte)0x7E, (byte)0x09, (byte)0x02,
        (byte)0x18, (byte)0xA4, (byte)0xA4, (byte)0x9C, (byte)0x78,
        (byte)0x7F, (byte)0x08, (byte)0x04, (byte)0x04, (byte)0x78,
        (byte)0x00, (byte)0x44, (byte)0x7D, (byte)0x40, (byte)0x00,
        (byte)0x20, (byte)0x40, (byte)0x40, (byte)0x3D, (byte)0x00,
        (byte)0x7F, (byte)0x10, (byte)0x28, (byte)0x44, (byte)0x00,
        (byte)0x00, (byte)0x41, (byte)0x7F, (byte)0x40, (byte)0x00,
        (byte)0x7C, (byte)0x04, (byte)0x78, (byte)0x04, (byte)0x78,
        (byte)0x7C, (byte)0x08, (byte)0x04, (byte)0x04, (byte)0x78,
        (byte)0x38, (byte)0x44, (byte)0x44, (byte)0x44, (byte)0x38,
        (byte)0xFC, (byte)0x18, (byte)0x24, (byte)0x24, (byte)0x18,
        (byte)0x18, (byte)0x24, (byte)0x24, (byte)0x18, (byte)0xFC,
        (byte)0x7C, (byte)0x08, (byte)0x04, (byte)0x04, (byte)0x08,
        (byte)0x48, (byte)0x54, (byte)0x54, (byte)0x54, (byte)0x24,
        (byte)0x04, (byte)0x04, (byte)0x3F, (byte)0x44, (byte)0x24,
        (byte)0x3C, (byte)0x40, (byte)0x40, (byte)0x20, (byte)0x7C,
        (byte)0x1C, (byte)0x20, (byte)0x40, (byte)0x20, (byte)0x1C,
        (byte)0x3C, (byte)0x40, (byte)0x30, (byte)0x40, (byte)0x3C,
        (byte)0x44, (byte)0x28, (byte)0x10, (byte)0x28, (byte)0x44,
        (byte)0x4C, (byte)0x90, (byte)0x90, (byte)0x90, (byte)0x7C,
        (byte)0x44, (byte)0x64, (byte)0x54, (byte)0x4C, (byte)0x44,
        (byte)0x00, (byte)0x08, (byte)0x36, (byte)0x41, (byte)0x00,
        (byte)0x00, (byte)0x00, (byte)0x77, (byte)0x00, (byte)0x00,
        (byte)0x00, (byte)0x41, (byte)0x36, (byte)0x08, (byte)0x00,
        (byte)0x02, (byte)0x01, (byte)0x02, (byte)0x04, (byte)0x02,
        (byte)0x3C, (byte)0x26, (byte)0x23, (byte)0x26, (byte)0x3C,
        (byte)0x1E, (byte)0xA1, (byte)0xA1, (byte)0x61, (byte)0x12,
        (byte)0x3A, (byte)0x40, (byte)0x40, (byte)0x20, (byte)0x7A,
        (byte)0x38, (byte)0x54, (byte)0x54, (byte)0x55, (byte)0x59,
        (byte)0x21, (byte)0x55, (byte)0x55, (byte)0x79, (byte)0x41,
        (byte)0x22, (byte)0x54, (byte)0x54, (byte)0x78, (byte)0x42, // a-umlaut
        (byte)0x21, (byte)0x55, (byte)0x54, (byte)0x78, (byte)0x40,
        (byte)0x20, (byte)0x54, (byte)0x55, (byte)0x79, (byte)0x40,
        (byte)0x0C, (byte)0x1E, (byte)0x52, (byte)0x72, (byte)0x12,
        (byte)0x39, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x59,
        (byte)0x39, (byte)0x54, (byte)0x54, (byte)0x54, (byte)0x59,
        (byte)0x39, (byte)0x55, (byte)0x54, (byte)0x54, (byte)0x58,
        (byte)0x00, (byte)0x00, (byte)0x45, (byte)0x7C, (byte)0x41,
        (byte)0x00, (byte)0x02, (byte)0x45, (byte)0x7D, (byte)0x42,
        (byte)0x00, (byte)0x01, (byte)0x45, (byte)0x7C, (byte)0x40,
        (byte)0x7D, (byte)0x12, (byte)0x11, (byte)0x12, (byte)0x7D, // A-umlaut
        (byte)0xF0, (byte)0x28, (byte)0x25, (byte)0x28, (byte)0xF0,
        (byte)0x7C, (byte)0x54, (byte)0x55, (byte)0x45, (byte)0x00,
        (byte)0x20, (byte)0x54, (byte)0x54, (byte)0x7C, (byte)0x54,
        (byte)0x7C, (byte)0x0A, (byte)0x09, (byte)0x7F, (byte)0x49,
        (byte)0x32, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x32,
        (byte)0x3A, (byte)0x44, (byte)0x44, (byte)0x44, (byte)0x3A, // o-umlaut
        (byte)0x32, (byte)0x4A, (byte)0x48, (byte)0x48, (byte)0x30,
        (byte)0x3A, (byte)0x41, (byte)0x41, (byte)0x21, (byte)0x7A,
        (byte)0x3A, (byte)0x42, (byte)0x40, (byte)0x20, (byte)0x78,
        (byte)0x00, (byte)0x9D, (byte)0xA0, (byte)0xA0, (byte)0x7D,
        (byte)0x3D, (byte)0x42, (byte)0x42, (byte)0x42, (byte)0x3D, // O-umlaut
        (byte)0x3D, (byte)0x40, (byte)0x40, (byte)0x40, (byte)0x3D,
        (byte)0x3C, (byte)0x24, (byte)0xFF, (byte)0x24, (byte)0x24,
        (byte)0x48, (byte)0x7E, (byte)0x49, (byte)0x43, (byte)0x66,
        (byte)0x2B, (byte)0x2F, (byte)0xFC, (byte)0x2F, (byte)0x2B,
        (byte)0xFF, (byte)0x09, (byte)0x29, (byte)0xF6, (byte)0x20,
        (byte)0xC0, (byte)0x88, (byte)0x7E, (byte)0x09, (byte)0x03,
        (byte)0x20, (byte)0x54, (byte)0x54, (byte)0x79, (byte)0x41,
        (byte)0x00, (byte)0x00, (byte)0x44, (byte)0x7D, (byte)0x41,
        (byte)0x30, (byte)0x48, (byte)0x48, (byte)0x4A, (byte)0x32,
        (byte)0x38, (byte)0x40, (byte)0x40, (byte)0x22, (byte)0x7A,
        (byte)0x00, (byte)0x7A, (byte)0x0A, (byte)0x0A, (byte)0x72,
        (byte)0x7D, (byte)0x0D, (byte)0x19, (byte)0x31, (byte)0x7D,
        (byte)0x26, (byte)0x29, (byte)0x29, (byte)0x2F, (byte)0x28,
        (byte)0x26, (byte)0x29, (byte)0x29, (byte)0x29, (byte)0x26,
        (byte)0x30, (byte)0x48, (byte)0x4D, (byte)0x40, (byte)0x20,
        (byte)0x38, (byte)0x08, (byte)0x08, (byte)0x08, (byte)0x08,
        (byte)0x08, (byte)0x08, (byte)0x08, (byte)0x08, (byte)0x38,
        (byte)0x2F, (byte)0x10, (byte)0xC8, (byte)0xAC, (byte)0xBA,
        (byte)0x2F, (byte)0x10, (byte)0x28, (byte)0x34, (byte)0xFA,
        (byte)0x00, (byte)0x00, (byte)0x7B, (byte)0x00, (byte)0x00,
        (byte)0x08, (byte)0x14, (byte)0x2A, (byte)0x14, (byte)0x22,
        (byte)0x22, (byte)0x14, (byte)0x2A, (byte)0x14, (byte)0x08,
        (byte)0x55, (byte)0x00, (byte)0x55, (byte)0x00, (byte)0x55, // #176 (25% block) missing in old code
        (byte)0xAA, (byte)0x55, (byte)0xAA, (byte)0x55, (byte)0xAA, // 50% block
        (byte)0xFF, (byte)0x55, (byte)0xFF, (byte)0x55, (byte)0xFF, // 75% block
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x00,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0xFF, (byte)0x00,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0xFF, (byte)0x00,
        (byte)0x10, (byte)0x10, (byte)0xFF, (byte)0x00, (byte)0xFF,
        (byte)0x10, (byte)0x10, (byte)0xF0, (byte)0x10, (byte)0xF0,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0xFC, (byte)0x00,
        (byte)0x14, (byte)0x14, (byte)0xF7, (byte)0x00, (byte)0xFF,
        (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x00, (byte)0xFF,
        (byte)0x14, (byte)0x14, (byte)0xF4, (byte)0x04, (byte)0xFC,
        (byte)0x14, (byte)0x14, (byte)0x17, (byte)0x10, (byte)0x1F,
        (byte)0x10, (byte)0x10, (byte)0x1F, (byte)0x10, (byte)0x1F,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0x1F, (byte)0x00,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0xF0, (byte)0x00,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x1F, (byte)0x10,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0x1F, (byte)0x10,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0xF0, (byte)0x10,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x10,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0x10, (byte)0x10,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0xFF, (byte)0x10,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x14,
        (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x00, (byte)0xFF,
        (byte)0x00, (byte)0x00, (byte)0x1F, (byte)0x10, (byte)0x17,
        (byte)0x00, (byte)0x00, (byte)0xFC, (byte)0x04, (byte)0xF4,
        (byte)0x14, (byte)0x14, (byte)0x17, (byte)0x10, (byte)0x17,
        (byte)0x14, (byte)0x14, (byte)0xF4, (byte)0x04, (byte)0xF4,
        (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x00, (byte)0xF7,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0x14, (byte)0x14,
        (byte)0x14, (byte)0x14, (byte)0xF7, (byte)0x00, (byte)0xF7,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0x17, (byte)0x14,
        (byte)0x10, (byte)0x10, (byte)0x1F, (byte)0x10, (byte)0x1F,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0xF4, (byte)0x14,
        (byte)0x10, (byte)0x10, (byte)0xF0, (byte)0x10, (byte)0xF0,
        (byte)0x00, (byte)0x00, (byte)0x1F, (byte)0x10, (byte)0x1F,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x1F, (byte)0x14,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFC, (byte)0x14,
        (byte)0x00, (byte)0x00, (byte)0xF0, (byte)0x10, (byte)0xF0,
        (byte)0x10, (byte)0x10, (byte)0xFF, (byte)0x10, (byte)0xFF,
        (byte)0x14, (byte)0x14, (byte)0x14, (byte)0xFF, (byte)0x14,
        (byte)0x10, (byte)0x10, (byte)0x10, (byte)0x1F, (byte)0x00,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xF0, (byte)0x10,
        (byte)0xFF, (byte)0xFF, (byte)0xFF, (byte)0xFF, (byte)0xFF,
        (byte)0xF0, (byte)0xF0, (byte)0xF0, (byte)0xF0, (byte)0xF0,
        (byte)0xFF, (byte)0xFF, (byte)0xFF, (byte)0x00, (byte)0x00,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0xFF,
        (byte)0x0F, (byte)0x0F, (byte)0x0F, (byte)0x0F, (byte)0x0F,
        (byte)0x38, (byte)0x44, (byte)0x44, (byte)0x38, (byte)0x44,
        (byte)0xFC, (byte)0x4A, (byte)0x4A, (byte)0x4A, (byte)0x34, // sharp-s or beta
        (byte)0x7E, (byte)0x02, (byte)0x02, (byte)0x06, (byte)0x06,
        (byte)0x02, (byte)0x7E, (byte)0x02, (byte)0x7E, (byte)0x02,
        (byte)0x63, (byte)0x55, (byte)0x49, (byte)0x41, (byte)0x63,
        (byte)0x38, (byte)0x44, (byte)0x44, (byte)0x3C, (byte)0x04,
        (byte)0x40, (byte)0x7E, (byte)0x20, (byte)0x1E, (byte)0x20,
        (byte)0x06, (byte)0x02, (byte)0x7E, (byte)0x02, (byte)0x02,
        (byte)0x99, (byte)0xA5, (byte)0xE7, (byte)0xA5, (byte)0x99,
        (byte)0x1C, (byte)0x2A, (byte)0x49, (byte)0x2A, (byte)0x1C,
        (byte)0x4C, (byte)0x72, (byte)0x01, (byte)0x72, (byte)0x4C,
        (byte)0x30, (byte)0x4A, (byte)0x4D, (byte)0x4D, (byte)0x30,
        (byte)0x30, (byte)0x48, (byte)0x78, (byte)0x48, (byte)0x30,
        (byte)0xBC, (byte)0x62, (byte)0x5A, (byte)0x46, (byte)0x3D,
        (byte)0x3E, (byte)0x49, (byte)0x49, (byte)0x49, (byte)0x00,
        (byte)0x7E, (byte)0x01, (byte)0x01, (byte)0x01, (byte)0x7E,
        (byte)0x2A, (byte)0x2A, (byte)0x2A, (byte)0x2A, (byte)0x2A,
        (byte)0x44, (byte)0x44, (byte)0x5F, (byte)0x44, (byte)0x44,
        (byte)0x40, (byte)0x51, (byte)0x4A, (byte)0x44, (byte)0x40,
        (byte)0x40, (byte)0x44, (byte)0x4A, (byte)0x51, (byte)0x40,
        (byte)0x00, (byte)0x00, (byte)0xFF, (byte)0x01, (byte)0x03,
        (byte)0xE0, (byte)0x80, (byte)0xFF, (byte)0x00, (byte)0x00,
        (byte)0x08, (byte)0x08, (byte)0x6B, (byte)0x6B, (byte)0x08,
        (byte)0x36, (byte)0x12, (byte)0x36, (byte)0x24, (byte)0x36,
        (byte)0x06, (byte)0x0F, (byte)0x09, (byte)0x0F, (byte)0x06,
        (byte)0x00, (byte)0x00, (byte)0x18, (byte)0x18, (byte)0x00,
        (byte)0x00, (byte)0x00, (byte)0x10, (byte)0x10, (byte)0x00,
        (byte)0x30, (byte)0x40, (byte)0xFF, (byte)0x01, (byte)0x01,
        (byte)0x00, (byte)0x1F, (byte)0x01, (byte)0x01, (byte)0x1E,
        (byte)0x00, (byte)0x19, (byte)0x1D, (byte)0x17, (byte)0x12,
        (byte)0x00, (byte)0x3C, (byte)0x3C, (byte)0x3C, (byte)0x3C,
        (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00  // #255 NBSP
    };

}
