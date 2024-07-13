import filter_tools
import numpy as np

f_max = 20 # Hz
f_sys = 100 # Hz
f_sample = 1000 # Hz
# f_sample = 1000 # Hz
filter_tools.check(f_max < f_sys < f_sample)

#######################
# ANALOG STAGE DESIGN #
#######################

analog_order = 2
wanted_analog_attenuation_at_fsample_over_2 = filter_tools.scaleToDb(1/20) # dB4
analog_filter = filter_tools.design_analog_filter(
    analog_order, wanted_analog_attenuation_at_fsample_over_2, f_sample, f_max,
    # resistors=filter_tools.E24, capacitors=filter_tools.C6,
    resistors=filter_tools.E24_minus_12, capacitors=filter_tools.C1,
    filter_name = 'butterworth',
    #filter_name = 'chebychev1', wanted_low_attenuation = .5 # dB
)

f_cut_a = analog_filter['f_cut_high_attenuation']
filter_tools.check(f_max < analog_filter['f_cut_low_attenuation'] < f_sample)
filter_tools.check(f_max < f_cut_a < f_sample)

########################
# DIGITAL STAGE DESIGN #
########################

digital_order = 5
float_type = np.float32
wanted_digital_attenuation_at_fsys_over_2 = filter_tools.scaleToDb(1/50)

digital_filter = filter_tools.design_digital_filter(
    digital_order, wanted_digital_attenuation_at_fsys_over_2 , f_max, f_sample, f_sys, float_type,
    filter_name = 'butterworth'
    #filter_name = 'chebychev2'
)

f_cut_d = digital_filter['f_cut_high_attenuation']
filter_tools.check(f_max < digital_filter['f_cut_low_attenuation'] < f_sys/2 )
filter_tools.check(f_max < f_cut_d < f_sys/2)

total_delay = analog_filter['max_delay'] + digital_filter['max_delay']
filter_tools.check(total_delay < filter_tools.minimal_human_percption_time)

print("###########\n")

print(f"total delay : {total_delay*1000} ms")
print(f"minimal human perception time : {filter_tools.minimal_human_percption_time*1000} ms")
