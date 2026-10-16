P0DIR   EQU 94h

    ORG   0d2eh
    LCALL 0a93h ; setup
    ANL P0DIR, #0C7h
    SETB  P0.0  
MAINLOOP:    
;; CLR P1.1???
    LCALL 03e1h ; read controller data
    LCALL READ_L2
    XRL   49h,#20h ; flip the vibrate button

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

ADCCON      EQU 0A1h
ADC_DATAH   EQU 0A2h
ADC_DATAL   EQU 0A3h
ADC_STATIC  EQU 0A4h
EXIF        EQU 91h
;P0DIR       EQU 94h

L2_CHANNEL  EQU 7           ; AIN7 = row 3 of the column driven by P0.4
SETTLE_CNT  EQU 100         ; about 77 us at 16 MHz

; in the main loop:
;     LCALL 03e1h
;     LCALL READ_L2

;; L2 was not sensitive enough, so read it manually
READ_L2:
    MOV   ADC_STATIC, #02Ah ; 10-bit, ADC clock = CPU/8 (as the matrix scan uses)
    MOV   ADCCON, #0A0h     ; ADC on, internal 1.22 V reference
    ORL   P0DIR, #040h      ; P0.6 input, as during the matrix scan
    LCALL SETTLE            ; ADC power-up
    ORL   P0, #010h         ; column P0.4 high
    LCALL SETTLE
    MOV   R4, #L2_CHANNEL
    LCALL ADC_CONVERT       ; dummy conversion: lets the mux settle
    LCALL ADC_CONVERT       ; real conversion
    MOV   A, ADC_DATAH      ; top 8 bits of the 10-bit result
    MOV   47h, A            ; payload byte 15
    ANL   P0, #0EFh         ; column low
    ANL   P0DIR, #0BFh      ; restore: P0.6 output, low, ADC off
    CLR   P0.6
    MOV   ADC_STATIC, #029h
    MOV   ADCCON, #000h
    RET

ADC_CONVERT:                ; R4 = channel
    ANL   EXIF, #0EFh       ; clear any stale end-of-conversion flag
    MOV   A, R4
    ORL   A, #0A0h
    MOV   ADCCON, A         ; CSTARTN = 1
    MOV   A, R4
    ORL   A, #020h
    MOV   ADCCON, A         ; CSTARTN = 0
    MOV   A, R4
    ORL   A, #0A0h
    MOV   ADCCON, A         ; CSTARTN = 1: rising edge starts the conversion
CONV_WAIT:
    MOV   A, EXIF
    JNB   ACC.4, CONV_WAIT  ; EXIF.4 = end of conversion
    ANL   EXIF, #0EFh
    RET

SETTLE:
    MOV   R2, #SETTLE_CNT
SETTLE_LP:
    DJNZ  R2, SETTLE_LP
    RET

    END