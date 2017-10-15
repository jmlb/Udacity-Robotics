
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered_outliers = outlier_filter.filter()