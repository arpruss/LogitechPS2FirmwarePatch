; --- equates, at the top of the file ---
ADCCON      EQU 0A1h
ADC_DATAH   EQU 0A2h
ADC_STATIC  EQU 0A4h
PWMCON      EQU 0a9h
EXIF        EQU 91h
P0DIR       EQU 94h
P0ALT       EQU 95h
P1DIR       EQU 96h
SETTLE_CNT  EQU 100          ; about 77 us at 16 MHz

    ORG   0h
    
    MOV   PSW,#0h
    MOV   SP,#08eh

    MOV   P0DIR,#07bh
    MOV   P1DIR,#0fdh
    CLR   P0.1
    MOV   P0DIR,#079h
    
    MOV   PWMCON,#0c1h
    ORL   P0ALT,#080h
    SETB  P0.7    

; --- main loop ---
MAINLOOP:
    LCALL READ_ALL           ; replaces LCALL 03e1h and READ_BTNS/READ_TRIGGERS

;    XRL   49h,#20h ; flip the vibrate button

    SETB  P1.1              ;  2 cycles ; high marker: 23 cycles
    MUL   AB                ;  5 cycle delay
    MUL   AB                ;  5 cycle delay
    MUL   AB                ;  5 cycle delay
    MOV   R0, #38H          ;  2 cycles ;  RAM start pointer (18 payload bytes)
    MOV   R2, #13H          ;  2 cycles ;  Total bytes to send = 19 (18 payload + 1 checksum)
    CLR   A                 ;  1 cycle ; Clear ACC
    MOV   R3, A             ;  1 cycle ; Clear running checksum

    CLR   P1.1              ;  2 cycles ; Line goes LOW (Marker / Inter-byte low) for at least 17 cycles 
    MUL   AB                ;  5 cycle delay
    MUL   AB                ;  5 cycle delay

LOAD_BYTE:
    MOV   A, @R0            ;  1 cycle ; Fetch byte from RAM
    ADD   A, R3             ;  1 cycle ; Accumulate running checksum
    MOV   R3, A             ;  1 cycle ; Store running checksum
    MOV   A, @R0            ;  1 cycle ; Reload original payload byte
    INC   R0                ;  1 cycle ; Advance RAM pointer

SEND_ACC:
    CLR   P1.1              ;  2 cycles ; Line goes LOW (Marker / Inter-byte low) for at least 5 cycles 
    MOV   R1, #08H          ;  2 cycles ; Reset bit counter

BIT_LOOP:
    RRC   A                 ;  1 cycle ; Shift LSB into Carry (C)
    SETB  P1.1              ;  2 cycles ; Line goes HIGH for 10 cycles if bit=1; 5 cycles otherwise
    JNC   BIT_LOW_PHASE     ;  3 cycles ; jump on zero

    NOP                     ;  Delay (1 cycle) for Bit=1
    NOP                     ;  Delay (1 cycle)
    NOP                     ;  Delay (1 cycle)
    NOP                     ;  Delay (1 cycle)
    NOP                     ;  Delay (1 cycle)

BIT_LOW_PHASE:
    CLR   P1.1              ;  2 cycles; Line goes LOW for at least 5 cycles 

    DJNZ  R1, BIT_LOOP      ;  3 cycles ; Loop 8 bits

    ; --- Inter-Byte Routing ---
    DJNZ  R2, CHECK_BYTE    ;  Decrement byte count (19 -> 18 ... -> 0)
    SJMP  MAINLOOP          ;  Exit when R2 reaches 0 (after 19th byte)

CHECK_BYTE:
    CJNE  R2, #01H, LOAD_BYTE ;  If R2 > 1, fetch next payload byte
    MOV   A, R3               ;  If R2 == 1, load checksum into ACC for 19th pass
    SJMP  SEND_ACC            ;  Transmit checksum byte via SEND_ACC

; --- subroutines, after your last SJMP and before END ---
READ_ALL:
    ANL   P0DIR, #0C7h       ; P0.3-P0.5 outputs (column drivers)
    ORL   P0DIR, #040h       ; P0.6 input during the matrix scan, as in the OEM
    MOV   ADC_STATIC, #02Ah  ; 10-bit, ADC clock = CPU/8
    MOV   ADCCON, #0A0h      ; ADC on, internal 1.22 V reference
    MOV   49h, #0FFh         ; digital byte, all released
    LCALL SETTLE             ; ADC power-up
    MOV   R1, #0             ; column 0..2

COL_LOOP:
    MOV   A, R1
    MOV   DPTR, #COLUMN_MASK_TABLE
    MOVC  A, @A+DPTR         ; OEM column mask table: 08h, 10h, 20h
    MOV   R6, A
    ORL   P0, A              ; column high
    LCALL SETTLE
    MOV   R3, #0             ; row 0..3

ROW_LOOP:
    MOV   A, R3
    ADD   A, #4
    MOV   R4, A              ; ADC channel = 4 + row
    LCALL ADC_CONVERT        ; dummy: lets the mux settle
    LCALL ADC_CONVERT
    MOV   A, R1
    RL    A
    RL    A                  ; column * 4
    ADD   A, R3              ; matrix position
    MOV   DPTR, #PAYLOAD_INDEX_TABLE
    MOVC  A, @A+DPTR         ; OEM table: payload index - 5
    ADD   A, #3Dh            ; RAM address of that payload byte
    MOV   R0, A
    MOV   @R0, ADC_DATAH     ; raw top 8 bits of the 10-bit result
    INC   R3
    CJNE  R3, #4, ROW_LOOP

    ; digital buttons on this column
    MOV   A, R1
    MOV   DPTR, #ROW_MASK_TABLE_P10
    MOVC  A, @A+DPTR         ; P1.0 row masks: 01h select, 20h vibration, 02h L3
    JNB   P1.0, D10_END
;    CJNE  A, #020h, D10_CLR
;    ORL   49h, A             ; vibration: active high
;    SJMP  D10_END
;D10_CLR:
    CPL   A
    ANL   49h, A             ; pressed = bit cleared
D10_END:
    MOV   A, R1
    MOV   DPTR, #ROW_MASK_TABLE_P12
    MOVC  A, @A+DPTR         ; P1.2 row masks: 08h start, 10h mode, 04h R3
    JNB   P1.2, D12_END
    CPL   A
    ANL   49h, A
D12_END:
    MOV   A, R6
    CPL   A
    ANL   P0, A              ; column low
    INC   R1
    CJNE  R1, #3, COL_LOOP

    ; sticks: 8-bit, P0.6 driven high, as in the OEM
    ANL   P0DIR, #0BFh       ; P0.6 output
    SETB  P0.6
    MOV   ADC_STATIC, #029h  ; 8-bit
    MOV   ADCCON, #0A0h
    LCALL SETTLE
    MOV   R4, #3             ; AIN3 -> payload 2
    LCALL ADC_CONVERT
    LCALL ADC_CONVERT
    MOV   3Ah, ADC_DATAH
    MOV   R4, #2             ; AIN2 -> payload 1
    LCALL ADC_CONVERT
    LCALL ADC_CONVERT
    MOV   39h, ADC_DATAH
    MOV   R4, #1             ; AIN1 -> payload 4
    LCALL ADC_CONVERT
    LCALL ADC_CONVERT
    MOV   3Ch, ADC_DATAH
    MOV   R4, #0             ; AIN0 -> payload 3
    LCALL ADC_CONVERT
    LCALL ADC_CONVERT
    MOV   3Bh, ADC_DATAH
    CLR   P0.6
    MOV   ADCCON, #000h      ; ADC off
    RET

ADC_CONVERT:                 ; R4 = channel
    ANL   EXIF, #0EFh        ; clear any stale end-of-conversion flag
    MOV   A, R4
    ORL   A, #0A0h
    MOV   ADCCON, A          ; CSTARTN = 1
    MOV   A, R4
    ORL   A, #020h
    MOV   ADCCON, A          ; CSTARTN = 0
    MOV   A, R4
    ORL   A, #0A0h
    MOV   ADCCON, A          ; CSTARTN = 1: rising edge starts the conversion
CONV_WAIT:
    MOV   A, EXIF
    JNB   ACC.4, CONV_WAIT   ; EXIF.4 = end of conversion
    ANL   EXIF, #0EFh
    RET

SETTLE:
    MOV   R2, #SETTLE_CNT
SETTLE_LP:
    DJNZ  R2, SETTLE_LP
    RET

PAYLOAD_INDEX_TABLE:
    DB 04h,06h,05h,07h
    DB 09h,0bh,08h,0ah
    DB 00h,01h,02h,03h

COLUMN_MASK_TABLE:
    DB 08h,10h,20h

ROW_MASK_TABLE_P10:
    DB 01h,20h,02h
    
ROW_MASK_TABLE_P12:
    DB 08h,10h,04h

    END