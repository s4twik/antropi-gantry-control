# System block diagram — Mermaid source

Updated source based on the supplied overview. Page 1 of circuit.pdf retains
the overview image; page 2 gives the electrical and communication qualifications.

```mermaid
flowchart LR
    subgraph Power
        AC[220 V AC mains]
        PSU[24 V DC supply]
        REG[Regulated 5 V controller supply]
    end
    subgraph Control
        MCU[STM32 Nucleo-F446RE<br/>3.3 V logic]
        LD[3.3 V to 5 V differential line driver]
        OUT[Protected 24 V output stages]
        MSG[Mock message adapter<br/>actor, command, request ID]
    end
    subgraph Motion
        DRV[Delta ASD-A3-0721-L<br/>servo drive]
        MOT[Delta ECM-A3L 750 W<br/>AC servo motor]
        GAN[Gantry axis, 3 m rail<br/>35 kg moving mass]
    end
    subgraph Field["Field interface, 24 V via opto"]
        SEN[Home sensor NO<br/>End limits NC]
        PANEL[Operator panel<br/>Start, Reset]
    end
    subgraph External
        ROB[UR5 robot]
        CNC1[CNC lathe 1]
        CNC2[CNC lathe 2]
    end
    subgraph Safety
        EST[E-stop chain + contactor<br/>hardware, cuts drive power]
    end

    AC --> DRV
    AC --> PSU
    PSU --> REG
    REG -->|5 V power input| MCU
    REG --> LD
    PSU --> OUT
    PSU --> SEN
    PSU --> PANEL
    MCU -->|3.3 V pulse and direction| LD
    LD -->|5 V differential| DRV
    MCU -->|3.3 V outputs| OUT
    OUT -->|servo-ON, 24 V| DRV
    DRV -->|U V W power| MOT
    MOT --> GAN
    MOT -.->|encoder, loop inside drive| DRV
    DRV -->|ready, in-position, alarm<br/>24 V via opto| MCU
    SEN -->|24 V via opto<br/>polarity depends on NO or NC| MCU
    PANEL -->|24 V via opto| MCU
    MCU <--> MSG
    MSG <-.->|command and matching DONE<br/>transport not implemented| ROB
    ROB -->|travel pose, 24 V via opto| MCU
    OUT -->|access request, 24 V| CNC1
    MSG <-.->|command and matching DONE<br/>transport not implemented| CNC1
    OUT -->|access request, 24 V| CNC2
    MSG <-.->|command and matching DONE<br/>transport not implemented| CNC2
    CNC1 -->|maintained access grant, 24 V via opto| MCU
    CNC2 -->|maintained access grant, 24 V via opto| MCU
    OUT -->|status beacon, 24 V| PANEL
    EST -.->|removes 220 V| DRV
    EST -->|chain status, monitor only| MCU
```

The E-stop block is conceptual independent hardware, not a validated stop design.
No direct 24 V connection to MCU pins is intended. Field/input isolation does not
establish whole-system isolation; power, output and communication interfaces need
a defined grounding and isolation design.
