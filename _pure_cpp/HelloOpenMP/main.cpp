#include <omp.h>
#include <stdio.h>
#include <windows.h>
#include <math.h>


void test_tid()
{
	int nthreads, tid;
	/* Fork a team of threads with each thread having a private tid variable */
#pragma omp parallel num_threads(100) private(tid)
	{

		/* Obtain and print thread id */
		tid = omp_get_thread_num();
		printf("Hello World from thread = %d\n", tid);

		/* Only master thread does this */
		if (tid == 0) 
		{
			nthreads = omp_get_num_threads();
			printf("Number of threads = %d\n", nthreads);
		}

	}  /* All threads join master thread and terminate */
}

int main()
{
	DWORD time = GetTickCount();
	for (int k=0;k<10;k++)
	{	
		int i=0;
		float sum[100];
#pragma omp parallel num_threads(100) default(shared) shared(sum) private(i)
		{			
#pragma omp for		
			for (i=0;i<1000000;i++)
			{
				sum[i/10000] = sinf(i)*cosf(i);			
			}
		}
	}
	DWORD delta = GetTickCount() - time;
	printf("%d\n", delta/10);

	time = GetTickCount();
	for (int k=0;k<10;k++)
	{
		int i=0;
		float sum[100];
		for (int i=0;i<1000000;i++)
		{
			sum[i/10000] = sinf(i)*cosf(i);			
		}
	}
	delta = GetTickCount() - time;
	printf("%d\n", delta/10);

	test_tid();

	return 0;
}