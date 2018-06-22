; Copyright (C) 2014-2018 Joonas Javanainen <joonas.javanainen@gmail.com>
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
; SOFTWARE.

; Tests banking behaviour of a MBC1 cart with a 8 Mbit ROM

; Results have been verified with MBC1 research

.define CART_ROM_BANKS 64

.include "harness/mbc1_rom.s"

.bank 0 slot 0
.section "expected banks" FREE

expected_banks:

; $0000-$3FFF area, mode 0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0

; $0000-$3FFF area, mode 1
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32
.db  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
.db  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32
.db  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32  32

; $4000-$7FFF area
.db   1   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
.db  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31
.db  33  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47
.db  48  49  50  51  52  53  54  55  56  57  58  59  60  61  62  63
.db   1   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
.db  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31
.db  33  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47
.db  48  49  50  51  52  53  54  55  56  57  58  59  60  61  62  63

.ends
