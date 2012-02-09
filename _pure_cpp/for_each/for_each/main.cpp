#include <vector>
#include <list>
#include <algorithm>
#include <functional>
#include <numeric>

using namespace std;

template <typename InputIterator, typename OutputIterator, typename Predicate>
OutputIterator copy_if(InputIterator first, InputIterator last,
					   OutputIterator result, Predicate pred)
{
	while(first!=last)
	{
		if(pred(*first))
			*result++ = *first;
		++first;
	}
	return result;
}

class Point3i
{
	int _x,_y,_z;
public:
	Point3i(int x, int y, int z)
	{
		_x = x;
		_y = y;
		_z = z;
	}
	Point3i& operator=(const Point3i& rhs)
	{
		_x = rhs._x;
		_y = rhs._y;
		_z = rhs._z;
		return *this;
	}
	void draw()
	{
		printf("%d, %d, %d\n",_x,_y,_z);
	}
 
	bool operator==(const Point3i& rhs) const
	{
		return _x == rhs._x && _y == rhs._y && _y == rhs._y;
	}

	bool greaterByX(int n) const
	{
		return _x > n;
	}

	bool equalsByX(int n)
	{
		return _x == n;
	}

	bool sortByX(const Point3i& rhs) const
	{
		return _x < rhs._x;
	}
};

template <typename T>
void print_container(T& Container, const char* before="")
{
	printf("\n%s\n",before);
	for_each(Container.begin(), Container.end(), 
		mem_fun_ref(&Point3i::draw));
}

typedef vector<Point3i>::iterator Itr;

int main()
{
	vector<Point3i> points;
	for (int i=0;i<20;i++)
		points.push_back(Point3i(rand()%10,rand()%10,rand()%10));
	print_container(points, "push_back rand()%10");

	sort(points.begin(), points.end(), mem_fun_ref(&Point3i::sortByX));
	print_container(points, "std::sort Point3i::sortByX");

	points.erase(remove_if(points.begin(), points.end(),
		bind2nd(mem_fun_ref(&Point3i::equalsByX), 5)),
		points.end());
	print_container(points, "std::erase-remove_if Point3i::equalsByX(5)");

	vector<Point3i> pointsB;
	copy(points.begin(), points.end(), back_inserter(pointsB));
	print_container(pointsB, "std::copy");

	vector<Point3i> pointsGreaterThanFive;
	copy_if(points.begin(), points.end(), //src
		back_inserter(pointsGreaterThanFive),//dest
		bind2nd(mem_fun_ref(&Point3i::greaterByX), 5));//condition
	print_container(pointsGreaterThanFive, "std::copy_if Point3i::greaterByX(5)");

	printf("\nstd::find\n");
	Itr it = find(points.begin(), points.end(), points[1]);
	if (it != points.end())
		it->draw();

	char* foo[] = {"10","20","30","40","50"};
	int bar[5];
	int sum;
	transform (foo, foo+5, bar, ptr_fun(atoi) );//what happens if i remove ptr_fun
	sum = accumulate (bar, bar+5, 0);
	printf("\nsum = %d\n", sum);
	return 0;
}