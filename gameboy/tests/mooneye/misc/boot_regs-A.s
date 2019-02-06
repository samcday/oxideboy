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

; Tests initial register values

; Verified results:
;   pass: AGB, AGS
;   fail: DMG, MGB, SGB, SGB2, CGB

.include "common.s"

; First, let's check SP since it's not part of the normal save_results
; mechanism
.define EXPECTED_SP $FFFE

  ld (sp_save), sp
  ld sp, $FFFE

  push af

  ld a, (sp_save)
  cp <EXPECTED_SP
  jr nz, invalid_sp

  ld a, (sp_save+1)
  cp >EXPECTED_SP
  jr nz, invalid_sp

  pop af

; Now, let's check all the other registers

  save_results
  assert_a $11
  assert_f $00
  assert_b $01
  assert_c $00
  assert_d $00
  assert_e $08
  assert_h $00
  assert_l $7C
  jp process_results

invalid_sp:
  test_failure_string "INVALID SP VALUE"

.ramsection "Test-State" slot 2
  sp_save dw
.ends
