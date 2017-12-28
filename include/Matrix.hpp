#ifndef MANTRIXMANIPULATION
#define MANTRIXMANIPULATION

#include <iostream>
namespace GPSCurriculumDesign {
#define MAXMEMORY 10240

#define INITIALERROR 1471
#define MEMORYOVERFLOW 1472
#define UNMATCHEDPULSIZE 1473
#define UNMATCHEDTYPE 1474
#define UNMATCHEDMULTIPLSIZE 1475
#define UNMATCHEDVECTORSIZE 1476
#define NOTVECTOR 1477
#define NOTSQUAREMATRIX 1478
#define CROSSVECTORSIZEOVERTHREE 1479
//void PrintError(int errorcode);

template<typename _T>
class Matrix {
private:
	
	int ROWS;
	int COLS;
	_T* DATA;

public:
	Matrix() : ROWS(0),COLS(0){}
	Matrix(int rows, int cols, double initial = 0) 
		:ROWS(rows), COLS(cols){
		int length = 0;
		int MatrixSize = rows * cols;

		length = sizeof(_T);
		if (MatrixSize * length < MAXMEMORY) {
			DATA = new _T[MatrixSize];
			initial = (_T)initial;
			for (int r = 0; r < rows; r++) {
				for (int c = 0; c < cols; c++) {
					*(DATA + r * COLS + c) = initial;
				}
			}
		}
		else {
			PrintError(MEMORYOVERFLOW);
		}
	}
	

public:
	void PrintError(int);
	Matrix operator +(Matrix<_T>& m);
	Matrix operator -(Matrix<_T>& m);
	Matrix operator *(Matrix<_T>& m);
	void operator =(Matrix<_T>& m);

	Matrix t();
	_T Dot(Matrix<_T>& m);
	Matrix Cross(Matrix<_T>& m);
	Matrix Inv();
	Matrix Dialoge(_T t);
	
	friend std::ostream& operator<<(std::ostream& out, Matrix<_T>& m) {
		for (int r = 0; r < m.ROWS; r++) {
			for (int c = 0; c < m.COLS; c++) {
				if (r == 0 && c == 0) out << "[";
				out << *(m.DATA + r * m.COLS + c);
				if (r == m.ROWS - 1 && c == m.COLS - 1) {
					out << "]";
					return out;
				}
				else {
					out << " ";
				}
			}
			out << std::endl;
		}
		return out;
	}

	void ChangeElement(int r,int c,_T e) {
		if (r > this->ROWS || c > this->COLS || r < 0 || c < 0) {
			PrintError(MEMORYOVERFLOW);
			return;
		}
		*(DATA + r * COLS + c) = e;
	}

	_T GetElement(int r, int c) {
		if (r > this->ROWS || c > this->COLS || r < 0 || c < 0) {
			PrintError(MEMORYOVERFLOW);
			return 0;
		}

		_T t = *(DATA + r * COLS + c);
		
		return t;
	}

};

template<typename _T>
void Matrix<_T>::PrintError(int) {
	std::cout << "ERROR" << std::endl;
}

template<typename _T>
Matrix<_T> Matrix<_T>::operator+(Matrix<_T>& m)
{
	if (this->ROWS != m.ROWS || this->COLS != m.COLS) {
		PrintError(UNMATCHEDPULSIZE);
		return Matrix();
	}
	Matrix<_T> temp(ROWS, COLS);
	for (int r = 0; r < ROWS; r++) {
		for (int c = 0; c < COLS; c++) {
			temp.ChangeElement(r, c, this->GetElement(r, c) + m.GetElement(r, c));
		}
	}
	return temp;
}


template<typename _T>
Matrix<_T> Matrix<_T>::operator-(Matrix<_T>& m)
{
	if (this->ROWS != m.ROWS || this->COLS != m.COLS) {
		PrintError(UNMATCHEDPULSIZE);
		return Matrix();
	}
	Matrix<_T> temp(ROWS, COLS);
	for (int r = 0; r < ROWS; r++) {
		for (int c = 0; c < COLS; c++) {
			temp.ChangeElement(r, c, this->GetElement(r, c) - m.GetElement(r, c));
		}
	}
	return temp;
}


template<typename _T>
void Matrix<_T>::operator=(Matrix<_T>& m)
{
	this->ROWS = m.ROWS;
	this->COLS = m.COLS;
	this->DATA = m.DATA;
}

template<typename _T>
Matrix<_T> Matrix<_T>::operator*(Matrix<_T>& m)
{
	if (this->COLS != m.ROWS) {
		PrintError(UNMATCHEDMULTIPLSIZE);
		return Matrix();
	}
	Matrix<_T> temp(this->ROWS, m.COLS);
	_T sum = 0;
	for (int r = 0; r < ROWS; r++) {
		for (int c = 0; c < m.COLS; c++) {

			for (int i = 0; i < COLS; i++) {
				sum += GetElement(r, i) * m.GetElement(i, c);
			}
			temp.ChangeElement(r, c, sum);
			sum = 0;
		}
	}
	return temp;
}

template<typename _T>
Matrix<_T> Matrix<_T>::t()
{
	Matrix<_T> temp(COLS,ROWS);
	for (int r = 0; r < ROWS; r++) {
		for (int c = 0; c < COLS; c++) {
			temp.ChangeElement(c, r, GetElement(r,c));
		}
	}
	return temp;
}

template<typename _T>
_T Matrix<_T>::Dot(Matrix<_T>& m)
{
	if ((this->COLS != 1 && this->ROWS != 1) && (m.COLS != 1 && m.ROWS != 1)) {
		PrintError(NOTVECTOR);
		return 0;
	}


	if (COLS * ROWS != m.COLS * m.ROWS) {
		PrintError(UNMATCHEDVECTORSIZE);
		return -1;
	}
	_T sum = 0;

	for (int c = 0; c < COLS * ROWS; c++) {
		
		sum += GetElement(0,c) * m.GetElement(0,c);
	}
	return sum;
}

template<typename _T>
Matrix<_T> Matrix<_T>::Cross(Matrix<_T>& m) {
	if ((this->COLS != 1 && this->ROWS != 1) && (m.COLS != 1 && m.ROWS != 1)) {
		PrintError(NOTVECTOR);
		return 0;
	}

	if (COLS * ROWS > 3 || m.COLS * m.ROWS > 3) {
		PrintError(CROSSVECTORSIZEOVERTHREE);
		return 0;
	}

	if (COLS * ROWS != m.ROWS * m.COLS) {
		PrintError(UNMATCHEDVECTORSIZE)
	}

	
	if (COLS * ROWS == 1) {
		Matrix<_T> temp;
		return temp;
	}

	Matrix<_T> temp(3, 1);
	if (COLS * ROWS == 2) {
		temp.ChangeElement(0, 2, GetElement(0, 0) * m.GetElement(0, 1) - GetElement(0, 1) * m.GetElement(0, 0));
	}
	else if (COLS * ROWS == 3) {
		temp.ChangeElement(0, 0, GetElement(0, 1) * m.GetElement(0, 2) - GetElement(0, 2) * m.GetElement(0, 1));
		temp.ChangeElement(0, 1, GetElement(0, 2) * m.GetElement(0, 0) - GetElement(0, 0) * m.GetElement(0, 2));
		temp.ChangeElement(0, 0, GetElement(0, 0) * m.GetElement(0, 1) - GetElement(0, 1) * m.GetElement(0, 0));
	}
	return temp;
}

template<typename _T>
Matrix<_T> Matrix<_T>::Inv()
{
	if (COLS != ROWS) {
		PrintError(NOTSQUAREMATRIX);
		return Matrix();
	}

	if (COLS == 0 && ROWS == 0) {
		return Matrix();
	}

	const int matrixSize = ROWS;
	const int matrixSizePow = matrixSize * matrixSize;
	int i, j, k, l, u, v;
	int* is = new int[matrixSize];
	int* js = new int[matrixSize];
	double* b = new double[matrixSizePow];
	double d, p;
	//double b[matrixSize][matrixSize];
	for (int r = 0; r < ROWS; r++) {
		for (int c = 0; c < COLS; c++) {
			b[r * matrixSize + c] = GetElement(r, c);
		}
	}

	for (k = 0; k < matrixSize; k++)
	{
		d = 0.0;
		for (i = k; i < matrixSize; i++)   /* 查找右下角方阵中主元素的位置 */
		{
			for (j = k; j < matrixSize; j++)
			{
				l = matrixSize*i + j;
				p = fabs(b[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d < DBL_EPSILON)   /* 主元素接近于0，矩阵不可逆 */
		{
			printf("Divided by 0 in MatrixInv!\n");
			exit(EXIT_FAILURE);
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
		{
			for (j = 0; j < matrixSize; j++)
			{
				u = k*matrixSize + j;
				v = is[k] * matrixSize + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
		{
			for (i = 0; i < matrixSize; i++)
			{
				u = i*matrixSize + k;
				v = i*matrixSize + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k*matrixSize + k;
		b[l] = 1.0 / b[l];  /* 初等行变换 */
		for (j = 0; j < matrixSize; j++)
		{
			if (j != k)
			{
				u = k*matrixSize + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i < matrixSize; i++)
		{
			if (i != k)
			{
				for (j = 0; j < matrixSize; j++)
				{
					if (j != k)
					{
						u = i*matrixSize + j;
						b[u] = b[u] - b[i*matrixSize + k] * b[k*matrixSize + j];
					}
				}
			}
		}
		for (i = 0; i < matrixSize; i++)
		{
			if (i != k)
			{
				u = i*matrixSize + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = matrixSize - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
	{
		if (js[k] != k)
		{
			for (j = 0; j < matrixSize; j++)
			{
				u = k*matrixSize + j;
				v = js[k] * matrixSize + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i < matrixSize; i++)
			{
				u = i*matrixSize + k;
				v = is[k] + i*matrixSize;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}



	Matrix<_T> temp(matrixSize, matrixSize);
	for (int r = 0; r < ROWS; r++) {
		for (int c = 0; c < COLS; c++) {
			temp.ChangeElement(r,c, (_T)b[r * matrixSize + c]);
		}
	}

	delete[] b;
	delete[] is;
	delete[] js;

	return temp;
}

template<typename _T>
Matrix<_T> Matrix<_T>::Dialoge(_T t)
{
	for (int i = 0; i < ROWS; i++) {
		*(DATA + i * COLS + i) = t;
	}
	return *this;
}


} //NAMESPACE END




#endif	//MANTRIXMANIPULATION