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

import android.util.Log;

import java.io.Closeable;
import java.io.IOException;

/**
 * Based off Adafruit_GFX Arduino library.
 */

public abstract class AdafruitGraphix {

    private final int CHAR_BLOCK_WIDTH = 5;
    private final int CHAR_BLOCK_HEIGHT = 8;
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

    /**
     * Helper class, use in try block to display
     */
    public class Draw implements Closeable {

        public Draw(boolean clearScreen) {
            if (clearScreen) {
                AdafruitGraphix.this.clearScreen();
            }
        }

        public Draw() {
            this(false);
        }

        @Override
        public void close() {
            AdafruitGraphix.this.display();
        }
    }

    /**
     * Initialize display to given width and height
     * @param w width
     * @param h height
     */
    protected AdafruitGraphix(int w, int h) {
        this.width = w;
        this.height = h;
        initialize();
    }

    /**
     * Begin a drawing block.
     * @param clearScreen If true, also clear screen
     * @return Draw object that updates display when done
     */
    public Draw begin(boolean clearScreen) {
        return new Draw(clearScreen);
    }

    public Draw begin() {
        return new Draw();
    }

    /**
     * Re-initialize display
     */
    public void initialize() {
        this.cursorX = 0;
        this.cursorY = 0;
        this.colorFG = WHITE;
        this.colorBG = BLACK;
        this.textSize = 1;
        this.wrap = true;
        this.clearScreen();
    }

    /**
     * Draw a pixel. Must be implemented
     * @param x Column left to right
     * @param y Row top to bottom
     * @param color Pixel color
     */
    public abstract void drawPixel(int x, int y, short color);

    /**
     * Refresh display. Must be implemented
     */
    public abstract void display();

    public void setFG(short fg) {
        this.colorFG = fg;
    }

    public void setBG(short bg) {
        this.colorBG = bg;
    }

    public void setTextSize(int size) {
        this.textSize = size;
    }

    public void setWrap(boolean wrap) {
        this.wrap = wrap;
    }

    public void setCursor(int x, int y) {
        this.cursorX = x;
        this.cursorY = y;
    }

    /**
     * Override to optimize how a vertical line is drawn
     *
     * @param x Starting x position, assumed to be left
     * @param y Starting y position, assumed to be top
     * @param h Height of line.
     * @param color Pixel color.
     */
    public void drawFastVLine(int x, int y, int h, short color) {
        drawLineHelper(x, y, x, y+h-1, color);
    }

    /**
     * Override to optimize how a horizontal line is drawn.
     *
     * @param x Starting x position, assumed to be left
     * @param y Starting y position, assumed to be top
     * @param w Width of line
     * @param color Pixel color
     */
    public void drawFastHLine(int x, int y, int w, short color) {
        drawLineHelper(x, y, x+w-1, y, color);
    }

    /**
     * Generic draw a line, from (x0,y0) to (x1, y1) of a given color
     * @param x0 Start
     * @param y0 Start
     * @param x1 End
     * @param y1 End
     * @param color Pixel color
     */
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
            drawLineHelper(x0, y0, x1, y1, color);
        }
    }


    /**
     * Basic draw a line in a non-optimal way.
     * @param x0 Start
     * @param y0 Start
     * @param x1 End
     * @param y1 End
     * @param color Pixel color
     */
    protected void drawLineHelper(int x0, int y0, int x1, int y1, short color) {
        boolean steep = Math.abs(y1 - y0) > Math.abs(x1 - x0);
        if (steep) {
            int t = x0;
            x0 = y0;
            y0 = t;
            t = x1;
            x1 = y1;
            y1 = t;
        }

        if (x0 > x1) {
            int t = x0;
            x0 = x1;
            x1 = t;
            t = y0;
            y0 = y1;
            y1 = t;
        }

        int dx, dy;
        dx = x1 - x0;
        dy = Math.abs(y1 - y0);

        int err = dx / 2;
        int ystep;

        if (y0 < y1) {
            ystep = 1;
        } else {
            ystep = -1;
        }

        for (; x0<=x1; x0++) {
            if (steep) {
                drawPixel(y0, x0, color);
            } else {
                drawPixel(x0, y0, color);
            }
            err -= dy;
            if (err < 0) {
                y0 += ystep;
                err += dx;
            }
        }

    }

    /**
     * Draw a rectangle by drawing lines
     * @param x top
     * @param y left
     * @param w width
     * @param h height
     * @param color pixel color
     */
    public void drawRect(int x, int y, int w, int h, short color) {
        drawFastHLine(x, y, w, color);
        drawFastHLine(x, y+h-1, w, color);
        drawFastVLine(x, y, h, color);
        drawFastVLine(x+w-1, y, h, color);
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

    public void drawBitmapOverlay(int x, int y, byte [] bitmap, int w, int h, short color) {
        int byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
        byte b = 0;

        for(int j=0; j<h; j++, y++) {
            for(int i=0; i<w; i++) {
                if ((i & 7) != 0) {
                    b <<= 1;
                } else {
                    b = bitmap[j * byteWidth + i / 8];
                }
                if ((b & 0x80) != 0) {
                    drawPixel(x+i, y, color);
                }
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
        int cc = (int)c;
        if (cc < 0 || cc > 255) {
            cc = 0;
        }
        cc *= CHAR_BLOCK_WIDTH;
        for (int i = 0; i < CHAR_BLOCK_WIDTH; i++) {
            // each column - one byte per column
            // LSB is bottom row, MSB is top row
            int line = font[cc+i] & 0xff;  // LSB is bottom, MSB is top
            for (int j = 0; j < CHAR_BLOCK_HEIGHT; j++) {
                short color = (line & 1) != 0 ? colorFG : colorBG;
                if (textSize == 1) {
                    drawPixel(x+i, y+j, color);
                } else {
                    fillRect(x+i*textSize, y+j*textSize, textSize, textSize, color);
                }
                line >>= 1;
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
