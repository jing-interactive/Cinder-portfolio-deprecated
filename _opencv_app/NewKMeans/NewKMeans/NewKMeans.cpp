extern "C"
{
#include "kmeans.h"
#include "hikmeans.h"
};
#include <iostream> 
#include <fstream>
#include "../../../_common/vOpenCV/OpenCV.h"
#include <opencv2/flann/flann.hpp>

#define DIMENSION 128

//#define CVKMEANS
#define HKMEANS
#define AKMEANS
#define VISUAL

using namespace cv;
using namespace std;

void help()
{
	std::cout << "\nThis program demonstrates kmeans clustering.\n"
		"It generates an image with random points, then assigns a random number of cluster\n"
		"centers and uses kmeans to move those cluster centers to their representitive location\n"
		"Call\n"
		"./kmeans\n" << std::endl;
}

namespace ir
{
	double akmeans( InputArray _data, int K,
		CV_OUT InputOutputArray _bestLabels,
		TermCriteria criteria, int attempts,
		int flags, OutputArray _centers = noArray() );
}

void
vl_constructor ()
{
	VlState * state ;
#if defined(DEBUG)
	printf("VLFeat constructor called\n") ;
#endif

	state = vl_get_state() ;

#if ! defined(VL_DISABLE_THREADS)
#if   defined(VL_THREADS_POSIX)
	{
		typedef void (*destructorType)(void * );
		pthread_key_create (&state->threadKey,
			(destructorType)
			vl_thread_specific_state_delete) ;
		pthread_mutex_init (&state->mutex, NULL) ;
		pthread_cond_init (&state->mutexCondition, NULL) ;
	}
#elif defined(VL_THREADS_WIN)
	InitializeCriticalSection (&state->mutex) ;
	state->tlsIndex = TlsAlloc () ;
#endif
#else
	vl_get_state()->threadState = vl_thread_specific_state_new() ;
#endif

	state->malloc_func  = malloc ;
	state->realloc_func = realloc ;
	state->calloc_func  = calloc ;
	state->free_func    = free ;
	state->printf_func  = printf ;

#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
	_vl_x86cpu_info_init (&state->cpuInfo) ;
#endif

#if defined(VL_OS_WIN)
	{
		SYSTEM_INFO info;
		GetSystemInfo (&info) ;
		state->numCPUs = info.dwNumberOfProcessors ;
	}
#elif defined(VL_OS_MACOSX) || defined(VL_OS_LINUX)
	state->numCPUs = sysconf(_SC_NPROCESSORS_ONLN) ;
#else
	state->numCPUs = 1 ;
#endif
	state->simdEnabled = VL_TRUE ;
	state->maxNumThreads = 1 ;
}

void plot_tree(Mat &points, const VlHIKMNode* node, Mat &img, Point* previous, int level)
{
	VlIKMFilt* filter = node->filter;
	VlHIKMNode ** children = node->children;

	for (int i=0;i<filter->K;i++)
	{
		vl_ikm_acc idx = filter->centers[i];
		float x = points.at<uchar>(idx,0);
		float y = points.at<uchar>(idx,1);
		Point ct(x, y);
		circle( img, ct, (10-level*2), CV_RGB(0,255,0), 1, CV_AA);
		//if (previous != NULL && level < 2)
		//	line(img, *previous, ct, vDefaultColor(level), 5-level*2);
		if (children != NULL)
			plot_tree(points, children[i], img, &ct, level+1);
	}
}


void plot_tree(Mat &points, const VlHIKMTree* tree, Mat &img)
{
	plot_tree(points, tree->root, img, NULL, 0);
}

void HKMeans( Mat &points, int K, int nleaves, Mat &img) 
{
	int dimension = points.cols;
	int N = points.rows;
	VlHIKMTree* tree;
	{
		int depth = VL_MAX(1, ceil (log ((float)nleaves) / log((float)K))) ;
		tree = vl_hikm_new(VL_IKM_ELKAN);
		//vl_hikm_set_verbosity(tree, 1);
		vl_hikm_init(tree, dimension, K, depth);
		vl_hikm_train(tree, points.ptr() ,N);

		int k = vl_hikm_get_K(tree);
		//for (int i=0;i<k;i++)
		{
			const VlHIKMNode* root = vl_hikm_get_root(tree);
			VlIKMFilt* filter = root->filter;
			for (int j=0;j<filter->K;j++)
			{
				vl_ikm_acc idx = filter->centers[j];
			}
			_VlHIKMNode ** children = root->children;
		}
	}
#ifdef VISUAL
	if (dimension != 2)
		return;
	int sz = img.elemSize();
	int szz = CV_ELEM_SIZE(DataType<Point_<float>>::type);
	img = Scalar::all(0);

	for(int i = 0; i < N; i++ )
	{
		float x = points.at<uchar>(i,0);
		float y = points.at<uchar>(i,1);
		Point ipt(x,y);
		circle( img, ipt, 2, CV_RGB(255,0,0), CV_FILLED, CV_AA );
	}
	// 	const VlHIKMNode* root = vl_hikm_get_root(tree);
	// 	VlIKMFilt* filter = root->filter;
	// 	for (int j=0;j<filter->K;j++)
	// 	{
	// 		vl_ikm_acc idx = filter->centers[j];  
	// 		float x = points.at<uchar>(idx,0);
	// 		float y = points.at<uchar>(idx,1);
	// 		Point ipt(x,y);
	// 		circle(img, ipt, 10, CV_RGB(0,255,0), 1, CV_AA );
	// 	}
	plot_tree(points, tree, img);

	imshow("vlfeat HKMeans", img);
#endif
	vl_hikm_delete(tree);
}

void IKmeans(Mat &points, int K, Mat &img ) 
{
	int dimension = points.cols;
	int N = points.rows;
	VlIKMFilt* tree;
	{
		tree = vl_ikm_new(VL_IKM_ELKAN);
		vl_ikm_set_verbosity(tree, 1);
		vl_ikm_init_rand_data(tree, points.ptr(), dimension, N, K);
		vl_ikm_train(tree, points.ptr() ,N);

		//for (int i=0;i<k;i++)
		{
			for (int j=0;j<tree->K;j++)
			{
				vl_ikm_acc ct = tree->centers[j];
			}
		}
	}
#ifdef VISUAL
	if (dimension != 2)
		return;
	//output
	img = Scalar::all(0);

	for(int i = 0; i < N; i++ )
	{
		float x = points.at<uchar>(i,0);
		float y = points.at<uchar>(i,1);
		Point ipt(x,y); 
		circle( img, ipt, 2, CV_RGB(255,0,0), CV_FILLED, CV_AA );
	}
	for (int j=0;j<tree->K;j++)
	{
		vl_ikm_acc idx = tree->centers[j]; 
		float x = points.at<uchar>(idx,0);
		float y = points.at<uchar>(idx,1);
		Point ipt(x,y);
		circle( img, ipt, 10, CV_RGB(0,255,0), 1, CV_AA );
	}

	vl_ikm_delete(tree);

	imshow("vlfeat IKmeans", img);
#endif
}

template <typename Distance>
int hierarchicalClustering2(const Mat& features, Mat& centers, const ::cvflann::KMeansIndexParams& params,
							Distance d = Distance())
{
	typedef typename Distance::ElementType ElementType;
	typedef typename Distance::ResultType DistanceType;

	::cvflann::Matrix<ElementType> m_features((ElementType*)features.ptr<ElementType>(0), features.rows, features.cols);

	::cvflann::Matrix<DistanceType> m_centers((DistanceType*)centers.ptr<DistanceType>(0), centers.rows, centers.cols);

	::cvflann::KMeansIndex<Distance> kmeans(m_features, params, d);
	kmeans.buildIndex();

	int clusterNum = kmeans.getClusterCenters(m_centers);
	return clusterNum;
}

int main( int /*argc*/, char** /*argv*/ )
{
	vl_constructor();

	const int MAX_CLUSTERS = 100;

	Mat img(256, 256, CV_8UC3);
	RNG rng(12345);

	double freq = cv::getTickFrequency();
	for (int sampleCount=10000;sampleCount<100000;sampleCount*=2)
	{
		printf("c: %d\n", sampleCount);
		int k, clusterCount = 100;
		const int dim = DIMENSION;
		Mat points_(sampleCount, 1, CV_8UC(dim));

		clusterCount = MIN(clusterCount, sampleCount);
		Mat centers(clusterCount, dim, CV_8UC1);

		/* generate random sample from multigaussian distribution */
		for( k = 0; k < clusterCount; k++ )
		{
			cv::Vec<float, dim> center;
			cv::Vec<float, dim> diff;
			for (int i=0;i<dim;i++)
			{
				center[i] = rng.uniform(0.0, (img.cols+img.rows)/2.0);
				diff[i] = rng.uniform(0.0,(img.cols+img.rows)*0.05);
			}
			center[0] = rng.uniform(0, img.cols);
			center[1] = rng.uniform(0, img.rows);

			Mat pointChunk = points_.rowRange(k*sampleCount/clusterCount,
				k == clusterCount - 1 ? sampleCount :
				(k+1)*sampleCount/clusterCount);
			rng.fill(pointChunk, CV_RAND_NORMAL, center, diff);
		}
		//randShuffle(points_, 1, &rng);
		Mat points = points_.reshape(1);

		int64 tm = cv::getTickCount();

		Mat fpoints;
		points.convertTo(fpoints, CV_32FC1);

#ifdef FLANN_KMEANS
		cvflann::KMeansIndexParams  param = cvflann::KMeansIndexParams(6,11, cvflann::FLANN_CENTERS_KMEANSPP);
		int real_K = hierarchicalClustering2<cvflann::L2<float>>(fpoints, centers, param);
		printf("%.1f ms\n", (cv::getTickCount()-tm)*1000/freq);

#ifdef VISUAL
		if (dim == 2)
		{
			img = Scalar::all(0);
			for(int i = 0; i < sampleCount; i++ )
			{
				float x = fpoints.at<float>(i,0);
				float y = fpoints.at<float>(i,1);
				Point ipt(x,y); 
				circle( img, ipt, 2, CV_RGB(255,0,0), CV_FILLED, CV_AA );
			}
			for(int i = 0; i < real_K; i++ )
			{
				float x = centers.at<float>(i,0);
				float y = centers.at<float>(i,1);
				Point ipt(x,y); 
				circle( img, ipt, 10, CV_RGB(0,255,0), 1, CV_AA );
			}
			imshow("flann kmeans", img);
		}
#endif
#endif

#ifdef HKMEANS
		tm = cv::getTickCount();
		HKMeans(points, 4, clusterCount, img); 
		printf("hkmeans: %.1f ms\n", (cv::getTickCount()-tm)*1000/freq);
#endif
#ifdef IKMEANS
		tm = cv::getTickCount();
		IKmeans(points, clusterCount, img);
		printf("IKmeans: %.1f ms\n", (cv::getTickCount()-tm)*1000/freq);
#endif

#ifdef CVKMEANS
		tm = cv::getTickCount();
		{
			Mat labels;
			cv::kmeans(fpoints, clusterCount, labels, 
				TermCriteria(CV_TERMCRIT_ITER,1,0),
				3, KMEANS_PP_CENTERS, centers);
#ifdef VISUAL
			if (dim == 2)
			{
				img = Scalar::all(0);
				for(int i = 0; i < sampleCount; i++ )
				{
					int clusterIdx = labels.at<int>(i);
					float x = fpoints.at<float>(i,0);
					float y = fpoints.at<float>(i,1);
					Point ipt(x,y); 
					circle( img, ipt, 2, CV_RGB(255,0,0), CV_FILLED, CV_AA );
				}
				for(int i = 0; i < clusterCount; i++ )
				{
					float x = centers.at<float>(i,0);
					float y = centers.at<float>(i,1);
					Point ipt(x,y); 
					circle( img, ipt, 10, CV_RGB(0,255,0), 1, CV_AA );
				}
				imshow("cv kmeans", img);
			}
#endif
		}
		printf("cv::kmeans: %.1f ms\n", (cv::getTickCount()-tm)*1000/freq);
#endif

#ifdef AKMEANS
		tm = cv::getTickCount();
		{
			Mat labels;
			ir::akmeans(fpoints, clusterCount, labels, 
				TermCriteria(),
				3, KMEANS_RANDOM_CENTERS, centers);
#ifdef VISUAL
			if (dim == 2)
			{
				img = Scalar::all(0);
				for(int i = 0; i < sampleCount; i++ )
				{
					int clusterIdx = labels.at<int>(i);
					float x = fpoints.at<float>(i,0);
					float y = fpoints.at<float>(i,1);
					Point ipt(x,y); 
					circle( img, ipt, 2, CV_RGB(255,0,0), CV_FILLED, CV_AA );
				}
				for(int i = 0; i < clusterCount; i++ )
				{
					float x = centers.at<float>(i,0);
					float y = centers.at<float>(i,1);
					Point ipt(x,y); 
					circle( img, ipt, 10, CV_RGB(0,255,0), 1, CV_AA );
				}
				imshow("akmeans", img);
			}
#endif
		}
		printf("ir::akmeans: %.1f ms\n", (cv::getTickCount()-tm)*1000/freq);
#endif

#ifdef VISUAL
		cv::waitKey();
#endif
	}	
	return 0;
}



////////////////////////////////////////// kmeans ////////////////////////////////////////////

namespace ir
{
	static void generateRandomCenter(const vector<Vec2f>& box, float* center, RNG& rng)
	{
		size_t j, dims = box.size();
		float margin = 1.f/dims;
		for( j = 0; j < dims; j++ )
			center[j] = ((float)rng*(1.f+margin*2.f)-margin)*(box[j][1] - box[j][0]) + box[j][0];
	}

#define CV_DECL_ALIGNED(x) __declspec(align(x))

	static inline float distance(const float* a, const float* b, int n)
	{
		int j = 0; float d = 0.f;
#if 1
		if( 1 )
		{
			float CV_DECL_ALIGNED(16) buf[4];
			__m128 d0 = _mm_setzero_ps(), d1 = _mm_setzero_ps();

			for( ; j <= n - 8; j += 8 )
			{
				__m128 t0 = _mm_sub_ps(_mm_loadu_ps(a + j), _mm_loadu_ps(b + j));
				__m128 t1 = _mm_sub_ps(_mm_loadu_ps(a + j + 4), _mm_loadu_ps(b + j + 4));
				d0 = _mm_add_ps(d0, _mm_mul_ps(t0, t0));
				d1 = _mm_add_ps(d1, _mm_mul_ps(t1, t1));
			}
			_mm_store_ps(buf, _mm_add_ps(d0, d1));
			d = buf[0] + buf[1] + buf[2] + buf[3];
		}
		else
#endif
		{
			for( ; j <= n - 4; j += 4 )
			{
				float t0 = a[j] - b[j], t1 = a[j+1] - b[j+1], t2 = a[j+2] - b[j+2], t3 = a[j+3] - b[j+3];
				d += t0*t0 + t1*t1 + t2*t2 + t3*t3;
			}
		}

		for( ; j < n; j++ )
		{
			float t = a[j] - b[j];
			d += t*t;
		}
		return d;
	}

	/*
	k-means center initialization using the following algorithm:
	Arthur & Vassilvitskii (2007) k-means++: The Advantages of Careful Seeding
	*/
	static void generateCentersPP(const Mat& _data, Mat& _out_centers,
		int K, RNG& rng, int trials)
	{
		int i, j, k, dims = _data.cols, N = _data.rows;
		const float* data = _data.ptr<float>(0);
		size_t step = _data.step/sizeof(data[0]);
		vector<int> _centers(K);
		int* centers = &_centers[0];
		vector<float> _dist(N*3);
		float* dist = &_dist[0], *tdist = dist + N, *tdist2 = tdist + N;
		double sum0 = 0;

		centers[0] = (unsigned)rng % N;

		for( i = 0; i < N; i++ )
		{
			dist[i] = distance(data + step*i, data + step*centers[0], dims);
			sum0 += dist[i];
		}

		for( k = 1; k < K; k++ )
		{
			double bestSum = DBL_MAX;
			int bestCenter = -1;

			for( j = 0; j < trials; j++ )
			{
				double p = (double)rng*sum0, s = 0;
				for( i = 0; i < N-1; i++ )
					if( (p -= dist[i]) <= 0 )
						break;
				int ci = i;
				for( i = 0; i < N; i++ )
				{
					tdist2[i] = std::min(distance(data + step*i, data + step*ci, dims), dist[i]);
					s += tdist2[i];
				}

				if( s < bestSum )
				{
					bestSum = s;
					bestCenter = ci;
					std::swap(tdist, tdist2);
				}
			}
			centers[k] = bestCenter;
			sum0 = bestSum;
			std::swap(dist, tdist);
		}

		for( k = 0; k < K; k++ )
		{
			const float* src = data + step*centers[k];
			float* dst = _out_centers.ptr<float>(k);
			for( j = 0; j < dims; j++ )
				dst[j] = src[j];
		}
	}


	double akmeans( InputArray _data, int K,
		InputOutputArray _bestLabels,
		TermCriteria criteria, int attempts,
		int flags, OutputArray _centers)
	{
		const int SPP_TRIALS = 3;
		Mat data = _data.getMat();
		int N = data.rows > 1 ? data.rows : data.cols;
		int dims = (data.rows > 1 ? data.cols : 1)*data.channels();
		int type = data.depth();

		attempts = std::max(attempts, 1);
		CV_Assert( data.dims <= 2 && type == CV_32F && K > 0 );

		_bestLabels.create(N, 1, CV_32S, -1, true);

		Mat _labels, best_labels = _bestLabels.getMat();
		if( flags & CV_KMEANS_USE_INITIAL_LABELS )
		{
			CV_Assert( (best_labels.cols == 1 || best_labels.rows == 1) &&
				best_labels.cols*best_labels.rows == N &&
				best_labels.type() == CV_32S &&
				best_labels.isContinuous());
			best_labels.copyTo(_labels);
		}
		else
		{
			if( !((best_labels.cols == 1 || best_labels.rows == 1) &&
				best_labels.cols*best_labels.rows == N &&
				best_labels.type() == CV_32S &&
				best_labels.isContinuous()))
				best_labels.create(N, 1, CV_32S);
			_labels.create(best_labels.size(), best_labels.type());
		}
		int* labels = _labels.ptr<int>();

		Mat centers(K, dims, type), old_centers(K, dims, type);
		vector<int> counters(K);
		vector<Vec2f> _box(dims);
		Vec2f* box = &_box[0];

		double best_compactness = DBL_MAX, compactness = 0;
		RNG& rng = theRNG();
		int a, iter, i, j, k;

		if( criteria.type & TermCriteria::EPS )
			criteria.epsilon = std::max(criteria.epsilon, 0.);
		else
			criteria.epsilon = FLT_EPSILON;
		criteria.epsilon *= criteria.epsilon;

		if( criteria.type & TermCriteria::COUNT )
			criteria.maxCount = std::min(std::max(criteria.maxCount, 2), 100);
		else
			criteria.maxCount = 100;

		if( K == 1 )
		{
			attempts = 1;
			criteria.maxCount = 2;
		}

		const float* sample = data.ptr<float>(0);
		for( j = 0; j < dims; j++ )//用于generateRandomCenter增加准确率
			box[j] = Vec2f(sample[j], sample[j]);

		for( i = 1; i < N; i++ )
		{
			sample = data.ptr<float>(i);
			for( j = 0; j < dims; j++ )
			{
				float v = sample[j];
				box[j][0] = std::min(box[j][0], v);
				box[j][1] = std::max(box[j][1], v);
			}
		}

		for( a = 0; a < attempts; a++ )//尝试attempts次
		{
			double max_center_shift = DBL_MAX;
			//每次的最大迭代数为criteria.maxCount
			for( iter = 0; iter < criteria.maxCount && max_center_shift > criteria.epsilon; iter++ )
			{
				swap(centers, old_centers);

				if( iter == 0 && (a > 0 || !(flags & KMEANS_USE_INITIAL_LABELS)) )
				{
					//两种赋初始值的方法
					if( flags & KMEANS_PP_CENTERS )
						generateCentersPP(data, centers, K, rng, SPP_TRIALS);
					else
					{
						for( k = 0; k < K; k++ )
							generateRandomCenter(_box, centers.ptr<float>(k), rng);
					}
				}
				else
				{
					if( iter == 0 && a == 0 && (flags & KMEANS_USE_INITIAL_LABELS) )
					{
						for( i = 0; i < N; i++ )
							CV_Assert( (unsigned)labels[i] < (unsigned)K );
					}

					// compute centers
					centers = Scalar(0);
					for( k = 0; k < K; k++ )
						counters[k] = 0;

					for( i = 0; i < N; i++ )
					{
						sample = data.ptr<float>(i);
						k = labels[i];//所属的中心
						float* center = centers.ptr<float>(k);
						for( j = 0; j <= dims - 4; j += 4 )
						{
							float t0 = center[j] + sample[j];
							float t1 = center[j+1] + sample[j+1];

							center[j] = t0;
							center[j+1] = t1;

							t0 = center[j+2] + sample[j+2];
							t1 = center[j+3] + sample[j+3];

							center[j+2] = t0;
							center[j+3] = t1;
						}
						for( ; j < dims; j++ )
							center[j] += sample[j];
						counters[k]++;
					}

					if( iter > 0 )
						max_center_shift = 0;

					for( k = 0; k < K; k++ )
					{
						float* center = centers.ptr<float>(k);

						if( counters[k] != 0 )//属于中心点k的元素非零
						{//计算平均值
							float scale = 1.f/counters[k];
							for( j = 0; j < dims; j++ )
								center[j] *= scale;
						}
						else//否则重新赋值
							generateRandomCenter(_box, center, rng);

						if( iter > 0 )
						{//计算centers在两次迭代之间的误差值，用于终止迭代
							double dist = 0;
							const float* old_center = old_centers.ptr<float>(k);
							for( j = 0; j < dims; j++ )
							{
								double t = center[j] - old_center[j];
								dist += t*t;
							}
							max_center_shift = std::max(max_center_shift, dist);
						}
					}
				}

#if 1
				//build kd forest
				::cvflann::Matrix<float> m_centers((float*)centers.ptr<float>(), centers.rows, centers.cols);
				::cvflann::KDTreeIndex<cvflann::L2<float> > kdForestIndex(m_centers, ::cvflann::KDTreeIndexParams(4));
				kdForestIndex.buildIndex();

				//query
				int knn = 1;//because we only need the nearest center
				::cvflann::Matrix<float> m_queries((float*)data.ptr<float>(), data.rows, data.cols);
				int *indices = labels;
				vector<float> dist(data.rows);
				::cvflann::Matrix<int> m_indices(indices, data.rows, knn);
				::cvflann::Matrix<float> m_dists(&dist[0], data.rows, knn);
				kdForestIndex.knnSearch(m_queries, m_indices, m_dists, knn, ::cvflann::SearchParams(64,0,false));

				compactness = 0;
				for(int i = 0; i < N; i++ )
				{
					int k_best = labels[i];
					double min_dist = dist[k_best];

					compactness += min_dist;
				}

#else
				// assign labels
				compactness = 0;
				for( i = 0; i < N; i++ )
				{
					//从k个中心中查找离data[i]最接近的中心，记录于labels[i]
					sample = data.ptr<float>(i);
					int k_best = 0;
					double min_dist = DBL_MAX;

					for( k = 0; k < K; k++ )
					{
						const float* center = centers.ptr<float>(k);
						double dist = distance(sample, center, dims);

						if( min_dist > dist )
						{
							min_dist = dist;
							k_best = k;
						}
					}

					compactness += min_dist;
					labels[i] = k_best;
				}
#endif
			}

			if( compactness < best_compactness )
			{
				best_compactness = compactness;
				if( _centers.needed() )
					centers.copyTo(_centers);
				_labels.copyTo(best_labels);
			}
		}

		return best_compactness;
	}
}//end of ir