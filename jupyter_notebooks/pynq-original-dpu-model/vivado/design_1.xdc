#PMOD1 motor-driver1
set_property PACKAGE_PIN H12 [get_ports PWM_0]
set_property PACKAGE_PIN E10 [get_ports PWM_1]
set_property PACKAGE_PIN D10 [get_ports PWM_2]
set_property PACKAGE_PIN C11 [get_ports PWM_3]
set_property PACKAGE_PIN B10 [get_ports gpio_rtl_0_tri_o[0]]

#PMOD2 motor-driver2
set_property PACKAGE_PIN J11 [get_ports gpio_rtl_0_tri_o[5]]
set_property PACKAGE_PIN J10 [get_ports gpio_rtl_0_tri_o[6]]
set_property PACKAGE_PIN H11 [get_ports gpio_rtl_0_tri_o[1]]

#PMOD3 infrared sensor
set_property PACKAGE_PIN AE12 [get_ports gpio_rtl_1_tri_i[0]]
set_property PACKAGE_PIN AF12 [get_ports gpio_rtl_1_tri_i[1]]

#PMOD4 debug(led-sw)
set_property PACKAGE_PIN AC12 [get_ports gpio_rtl_0_tri_o[2]]
set_property PACKAGE_PIN AD12 [get_ports gpio_rtl_0_tri_o[3]]
set_property PACKAGE_PIN AE10 [get_ports gpio_rtl_0_tri_o[4]]
set_property PACKAGE_PIN AF10 [get_ports gpio_rtl_1_tri_i[2]]


set_property IOSTANDARD LVCMOS33 [get_ports PWM_0]
set_property IOSTANDARD LVCMOS33 [get_ports PWM_1]
set_property IOSTANDARD LVCMOS33 [get_ports PWM_2]
set_property IOSTANDARD LVCMOS33 [get_ports PWM_3]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[0]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[1]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[2]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[3]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[4]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[5]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_0_tri_o[6]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_1_tri_i[0]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_1_tri_i[1]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_rtl_1_tri_i[2]]