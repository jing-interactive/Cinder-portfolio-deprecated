#include <stdio.h>
#include <utility>
#include <stdlib.h>

template <typename T>
void Qsort(T array[], int low, int high)
{
	if (low < high)
	{
		int m = low;//a moving marker
		for (int i=low+1;i<high;i++)
		{
			if (array[i] < array[low])
				std::swap<T>(array[++m], array[i]);
		}
		std::swap<T>(array[m], array[low]);
		Qsort<T>(array, low, m-1);
		Qsort<T>(array, m+1, high);
	}
}

template <typename T>
void print(T array[], int n)
{
	for (int i=0;i<n;i++)
	{
		printf("%d ", static_cast<int>(array[i]));
	}
	printf("\n");
}

template <typename T>
void Qsort(T array[], int n)
{
	Qsort(array,0,n);
}

template <typename T>
int Bsearch(T array[], int n, T value)
{
	int low = 0;
	int high = n-1;
	int m;

	while (low <= high)
	{
		m = (low+high)/2;
		if (array[m] < value)
			low = m+1;
		else if (array[m] > value)
			high = m-1;
		else
			return m;
	}
	return -1;
}

int main()
{
	int numbers[]={6,3,4,11,2,14,20,31,21};
	int n = (sizeof(numbers) / sizeof(numbers[0]));
	print(numbers, n);
	Qsort(numbers, n);
	print(numbers, n);

	int idx = Bsearch(numbers, n, 3);
	printf("idx of %d is %d\n", 3, idx);
	idx = Bsearch(numbers, n, 2);
	printf("idx of %d is %d\n", 2, idx);
	idx = Bsearch(numbers, n, 4);
	printf("idx of %d is %d\n", 4, idx);
}