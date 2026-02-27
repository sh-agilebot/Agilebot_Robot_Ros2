# definr IOType
uint8 DI = 1
uint8 DO = 2
uint8 UI = 3
uint8 UO = 4
uint8 RI = 5
uint8 RO = 6
uint8 GI = 7
uint8 GO = 8

# current IO type
uint8 io_type

# Port ids
uint16[] ports

# Port values   
uint8[] values

# (optional) Port descriptions
string[] descriptions
