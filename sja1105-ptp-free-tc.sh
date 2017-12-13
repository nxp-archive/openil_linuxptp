sja1105-tool config default ls1021atsn

# General Parameters Table
sja1105-tool config modify general-parameters-table switchid 0x0
sja1105-tool config modify general-parameters-table host_port 0x4
sja1105-tool config modify general-parameters-table send_meta0 0x0
sja1105-tool config modify general-parameters-table send_meta1 0x0
sja1105-tool config modify general-parameters-table mac_fltres0 0x000000000000
sja1105-tool config modify general-parameters-table mac_flt0 0xffffffffffff
sja1105-tool config modify general-parameters-table mac_fltres1 0x000000000000
sja1105-tool config modify general-parameters-table mac_flt1 0xffffffffffff
sja1105-tool config modify general-parameters-table ignore2stf 0x1

sja1105-tool config upload

echo "Configured sja1105 as a free running 2-step E2E transparent clock!"
