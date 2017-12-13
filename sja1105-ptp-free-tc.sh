sja1105-tool config default ls1021atsn

# General Parameters Table
sja1105-tool config modify general-parameters-table send_meta0 0x0
sja1105-tool config modify general-parameters-table send_meta1 0x0
sja1105-tool config modify general-parameters-table ignore2stf 0x1

sja1105-tool config upload

echo "Configured sja1105 as a free running 2-step E2E transparent clock!"
