passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
axis_min = -0.35
axis_max = 0.35
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()