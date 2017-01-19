onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -radix hexadecimal /cpu/clk
add wave -noupdate -radix hexadecimal /cpu/if_new_pc
add wave -noupdate -radix hexadecimal /cpu/ic_memresult
add wave -noupdate -radix hexadecimal /cpu/if_instruction
add wave -noupdate -radix hexadecimal /cpu/ic_stall
add wave -noupdate -radix hexadecimal /cpu/id_data_reg1
add wave -noupdate -radix hexadecimal /cpu/id_data_reg2
add wave -noupdate -radix hexadecimal /cpu/exec1/operand1
add wave -noupdate -radix hexadecimal /cpu/exec1/operand2
add wave -noupdate -radix hexadecimal /cpu/exec1/alu/src1
add wave -noupdate -radix hexadecimal /cpu/exec1/alu/src2
add wave -noupdate -radix hexadecimal /cpu/exec1/aluresult
add wave -noupdate -radix hexadecimal /cpu/id_dest_reg
add wave -noupdate -radix hexadecimal /cpu/ex_reg_to_mem
add wave -noupdate -radix hexadecimal /cpu/ex_result
add wave -noupdate -radix hexadecimal /cpu/dc_memresult
add wave -noupdate -radix hexadecimal /cpu/dc_wdata
add wave -noupdate -radix hexadecimal /cpu/wb_wdata
add wave -noupdate -radix hexadecimal /cpu/dc_stall
add wave -noupdate -radix hexadecimal /cpu/Dcache/do_write
add wave -noupdate -radix hexadecimal /cpu/Dcache/data_in
add wave -noupdate -radix hexadecimal /cpu/wb_regwrite
add wave -noupdate -radix hexadecimal /cpu/wb_wreg
add wave -noupdate -radix hexadecimal /cpu/mem_rw
add wave -noupdate -radix hexadecimal /cpu/mem_ack
add wave -noupdate -radix hexadecimal /cpu/mem_addr
add wave -noupdate -radix hexadecimal /cpu/mem_data_out
add wave -noupdate -radix hexadecimal /cpu/id_is_branch
add wave -noupdate -radix hexadecimal /cpu/id_branch_1
add wave -noupdate -radix hexadecimal /cpu/id_branch_2
add wave -noupdate -radix hexadecimal /cpu/id_bne
add wave -noupdate -radix hexadecimal /cpu/if_new_pc
add wave -noupdate -radix hexadecimal -radixshowbase 0 /cpu/id_mimmediat
add wave -noupdate -radix hexadecimal /cpu/addr_branch
add wave -noupdate -radix hexadecimal /cpu/decode/reg1_data
add wave -noupdate /cpu/forward_branch_src1
add wave -noupdate -radix hexadecimal /cpu/decode/reg2_data
add wave -noupdate /cpu/forward_branch_src2
add wave -noupdate -radix hexadecimal /cpu/forwarding/ex_mem_dest_reg
add wave -noupdate -radix hexadecimal /cpu/forwarding/if_id_branch_src2
add wave -noupdate -radix hexadecimal /cpu/forwarding/if_id_branch_src1
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {6488 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 252
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {5761 ps} {6645 ps}
