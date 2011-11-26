#ifdef _DEBUG
#pragma comment(lib, "pcl_surface-gd.lib")
#pragma comment(lib, "pcl_io-gd.lib")
#pragma comment(lib, "pcl_features-gd.lib")
#pragma comment(lib, "pcl_kdtree-gd.lib")
#pragma comment(lib, "pcl_common-gd.lib")
#pragma comment(lib, "pcl_search-gd.lib")
#pragma comment(lib, "pcl_filters-gd.lib")
#else
#pragma comment(lib, "pcl_surface.lib")
#pragma comment(lib, "pcl_io.lib")
#pragma comment(lib, "pcl_features.lib")
#pragma comment(lib, "pcl_kdtree.lib")
#pragma comment(lib, "pcl_common.lib")
#pragma comment(lib, "pcl_search.lib")
#pragma comment(lib, "pcl_filters.lib")
#endif

#ifdef _DEBUG
#pragma comment(lib, "opencv_core231d.lib")
#else
#pragma comment(lib, "opencv_core231.lib")
#endif