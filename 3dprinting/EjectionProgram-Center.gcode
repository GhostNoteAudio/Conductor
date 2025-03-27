; print progress 0% 1 min
M73 P0 R1
; acceleration limits
M201 X12000 Y12000 Z500 E5000
; max feedrate
M203 X500 Y500 Z20 E30
; Set Starting Acceleration ???
M204 P20000 R5000 T20000
; jerk limits
M205 X9.00 Y9.00 Z3.00 E2.50 ; sets the jerk limits, mm/sec

M106 S0
M106 P2 S0
M104 S40 ;set extruder temp to turn on the HB fan and prevent filament oozing from nozzle
M710 A1 S255 ;turn on MC fan by default(P1S)
M960 S5 P1 ; turn on logo lamp
G90
M220 S100 ;Reset Feedrate
M221 S100 ;Reset Flowrate
M73.2   R1.0 ;Reset left time magnitude
M1002 set_gcode_claim_speed_level : 5
M221 X0 Y0 Z0 ; turn off soft endstop to prevent protential logic problem
M204 S10000 ; init ACC set to 10m/s^2

;===== heatbed preheat ====================
M1002 gcode_claim_action : 2
M140 S20 ;set bed temp
M190 S20 ;wait for bed temp

;=============turn on fans to prevent PLA jamming=================
M106 P2 S100 ; turn on big fan ,to cool down toolhead

;===== prepare print temperature and material ==========
M104 S40 ;set extruder temp

; Move down slightly for added safety
G91 ; relative movement
G0 Z40 F1200
G90 ; Absolute movement

M975 S1 ; turn on
M73 P71 R1
G1 X60 F12000
G1 Y245
G1 Y265 F3000

M412 S1 ; ===turn on filament runout detection===

M109 S40 ; drop nozzle temp, make filament shink a bit
G92 E0
M73 P76 R1
G1 E-0.05 F300

M73 P80 R1
M106 P1 S0

;===== wipe nozzle ===============================
M1002 gcode_claim_action : 14
M975 S1
M106 S255
M109 S40

M106 S255 ; turn on fan (G28 has turn off fan)
M975 S1 ; turn on vibration supression

M106 P2 S100 ; turn on big fan ,to cool down toolhead

M104 S40 ; set extrude temp earlier, to reduce wait time

;===== nozzle load line ===============================

M975 S1
G90 ; Absolute positioning
M83 ; Extrusion relative mode
T1000 ; No idea
G1 X18.0 Y1.0 Z50 F18000 ; Move above start position
G1 E0.01 F300
G1 X20 E0.01 F1800
M400

;========turn off light and wait extrude temperature =============
M1002 gcode_claim_action : 0
M106 S0 ; turn off fan
M106 P2 S0 ; turn off big fan
M106 P3 S0 ; turn off chamber fan

M975 S1 ; turn on mech mode supression
G90
G21
M83 ; use relative distances for extrusion

M142 P1 R35 S40
M981 S1 P20000 ;open spaghetti detector
M73 P85 R0
; layer num/total_layer_count: 1/1
;M622.1 S1 ; for prev firware, default turned on
;M1002 judge_flag timelapse_record_flag
;M622 J1
 ; timelapse without wipe tower
;M971 S11 C10 O0
;M1004 S5 P1  ; external shutter

; update layer progress
M73 L1
M991 S0 P0 ;notify layer change
;M106 S0
;M106 P2 S0
M204 S500
M73 P86 R0
M73 P87 R0
M73 P88 R0
M73 P89 R0
M73 P90 R0
M106 S0
M106 P2 S0
M981 S0 P20000 ; close spaghetti detector
; FEATURE: Custom




; ================== ADD MOVES BEGIN ============================



; Actual push procedure
G0 Z70 F4000
G0 X128 Y128 F8000
G0 Z3 F4000
G0 Y0 F1200
G0 Z50 F4000
G0 Y220 F8000
G0 Z3 F4000
G0 Y0 F1200
G0 Z50 F4000
G0 X128 Y128 F8000

M400 P100 ; wait for motion to complete




; ================== ADD MOVES END ============================




; filament end gcode 
M106 P3 S0

M400 ; wait for buffer to clear
G92 E0 ; zero the extruder
G1 E-0.08 F1800 ; retract
G1 Z50 F900
G1 X65 Y245 F12000 ; move to safe pos 
M73 P91 R0
G1 Y265 F3000

G1 X65 Y245 F12000
G1 Y265 F3000
M140 S0 ; turn off bed
M106 S0 ; turn off fan
M106 P2 S0 ; turn off remote part cooling fan
M106 P3 S0 ; turn off chamber cooling fan

; pull back filament to AMS
M620 S255
G1 X20 Y50 F12000
G1 Y-3
T255
G1 X65 F12000
G1 Y265
G1 X100 F12000 ; wipe
M621 S255
M104 S0 ; turn off hotend

;M622.1 S1 ; for prev firware, default turned on
;M1002 judge_flag timelapse_record_flag
;M622 J1
;    M400 ; wait all motion done
;    M991 S0 P-1 ;end smooth timelapse at safe pos
;    M400 S3 ;wait for last picture to be taken
;M623; end of "timelapse_record_flag"

M400 ; wait all motion done
M17 S
M17 Z0.4 ; lower z motor current to reduce impact if there is something in the bottom
G1 Z100.2 F600
G1 Z98.2
M400 P100
M17 R ; restore z current

M220 S100  ; Reset feedrate magnitude
M201.2 K1.0 ; Reset acc magnitude
M73.2   R1.0 ;Reset left time magnitude
M1002 set_gcode_claim_speed_level : 0

M17 X0.8 Y0.8 Z0.5 ; lower motor current to 45% power
M73 P100 R0
