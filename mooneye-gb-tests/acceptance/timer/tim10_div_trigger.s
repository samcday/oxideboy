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

; This test verifies that the timer count changes are actually triggered
; by bit 5 going low when writing to the DIV register in 65536 Hz mode.
;
; 32 cycles after resetting the internal div counter, bit 5 of the
; internal div counter will have been set. Writing to the DIV register
; at this time will cause bit 5 to change from high to low which in
; turn triggers a timer increment.

; Verified results:
;   pass: DMG, MGB, SGB, SGB2, CGB, AGB, AGS
;   fail: -

.incdir "../../common"
.include "common.s"

test:
  di
  xor a
  ld b,4
  ldh (<IE), a
  ldh (<IF), a
  ldh (<DIV), a
  ld a, b
  ldh (<TIMA), a
  ldh (<TMA),a
  ld a, %00000110 ; Start 65536 Hz timer (64 cycles)
  ldh (<TAC), a
  ld a,b
  ldh (<DIV),a
  nop
  ldh (<TIMA), a
  nop
  ldh (<DIV),a
  nops 12
  ldh a,(<TIMA)
  ld d,a

  ld a,b
  ldh (<TIMA), a
  ldh (<DIV),a
  nop
  ldh (<TIMA), a
  nop
  ldh (<DIV),a
  nops 13
  ldh a,(<TIMA)
  ld e,a

  save_results
  assert_d $05
  assert_e $06
  jp process_results

