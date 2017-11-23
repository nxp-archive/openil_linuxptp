sja1105-tool config default ls1021atsn

# General Parameters Table

sja1105-tool config modify general-parameters-table switchid 0x0
sja1105-tool config modify general-parameters-table host_port 0x4
sja1105-tool config modify general-parameters-table send_meta0 0x1
sja1105-tool config modify general-parameters-table mac_fltres0 0x011b19000000
sja1105-tool config modify general-parameters-table mac_flt0 0xffffffffffff
sja1105-tool config modify general-parameters-table ignore2stf 0x0

# AVB Parameters Table
sja1105-tool config modify avb-parameters-table entry-count 1
sja1105-tool config modify avb-parameters-table srcmeta 0x222222222222
sja1105-tool config modify avb-parameters-table destmeta 0x011b19000000

sja1105-tool config upload
#sja1105-tool config save sja1105-2-step-e2e-tc.xml
