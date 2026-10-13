    ORG   0d2eh
    LCALL 0a93h ; setup
MAINLOOP:    
    LCALL 03e1h ; read controller data
    
SEND_PACKET_32B:
    SETB  P1.1              ;  Start preamble HIGH
    MOV   R0, #38H          ;  RAM start pointer (18 payload bytes)
    MOV   R2, #13H          ;  Total bytes to send = 19 (18 payload + 1 checksum)
    CLR   A                 ;  Clear ACC
    MOV   R3, A             ;  Clear running checksum

LOAD_BYTE:
    MOV   A, @R0            ;  Fetch byte from RAM
    ADD   A, R3             ;  Accumulate running checksum
    MOV   R3, A             ;  Store running checksum
    MOV   A, @R0            ;  Reload original payload byte
    INC   R0                ;  Advance RAM pointer

SEND_ACC:
    CLR   P1.1              ;  Line goes LOW (Marker / Inter-byte low) for 4 cycles
    MOV   R1, #08H          ;  Reset bit counter (8 bits) (1 cycle)
    NOP                     ;  Delay (1 cycle)

BIT_LOOP:
    RRC   A                 ;  Shift LSB into Carry (C) (1 cycle)
    SETB  P1.1              ;  Line goes HIGH for 5 cycles = 3.75 us nominal (measured 4.25) if bit=1
    JNC   BIT_LOW_PHASE     ;  Jump if C=0 (2 cycles), to go high for 3 cycles = 2.25 us nominal if bit=0 

    NOP                     ;  Delay (1 cycle) for Bit 1 = 4 us HIGH)
    NOP                     ;  Delay (1 cycle)

BIT_LOW_PHASE:
    CLR   P1.1              ;  (1 cycle) Line goes LOW for at least 4 cycles 

    DJNZ  R1, BIT_LOOP      ;  Loop 8 bits (2 cycles) 

    ; --- Inter-Byte Routing ---
    DJNZ  R2, CHECK_BYTE    ;  Decrement byte count (19 -> 18 ... -> 0)
    SJMP  MAINLOOP          ;  Exit when R2 reaches 0 (after 19th byte)

CHECK_BYTE:
    CJNE  R2, #01H, LOAD_BYTE ; If R2 > 1, fetch next payload byte
    MOV   A, R3               ;  If R2 == 1, load checksum into ACC for 19th pass
    SJMP  SEND_ACC            ;  Transmit checksum byte via SEND_ACC
    END