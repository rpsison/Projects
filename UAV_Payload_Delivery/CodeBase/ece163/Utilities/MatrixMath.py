"""Matrix Library in Native Python, using lists of lists as a row-major representation of matrices.
   That is: [[1,2,3],[4,5,6]] is a 2x3 matrix and they are indexed from [0]. Thus A[1][2] is 6, and A[0][1] is 2."""
import math

def multiply(A,B):
    """
    Simple Matrix multiplication, raises arithmetic error if inner dimensions don't match

    :param A: matrix (list of lists) of [m x n]
    :param B: matrix (list of lists) of [n x r]
    :return: A*B matrix of [m x r]
    """
    if len(A[0]) != len(B):
        raise ArithmeticError('Inner dimensions do not match')
    result = [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]
    return result

def transpose(A):
    """
    Matrix transpose, swaps rows and columns

    :param A: Matrix (lists of list) of [m x n]
    :return: A' matrix [n x m]
    """
    result = [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]
    return result

def add(A,B):
    """
    Matrix addition, raises arithmetic error if matrix dimensions don't match

    :param A: matrix (list of lists) [m x n]
    :param B: matrix (list of lists) [m x n]
    :return: A+B [m x n]
    """
    if any([len(A) != len(B), len(A[0]) != len(B[0])]):
        raise ArithmeticError('Matrices do not have same dimension')
    result = [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    return result

def subtract(A,B):
    """
    Matrix subtraction, raises arithmetic error if matrix dimensions don't match

    :param A: matrix (list of lists) [m x n]
    :param B: matrix (list of lists) [m x n]
    :return: A-B [m x n]
    """
    if any([len(A) != len(B), len(A[0]) != len(B[0])]):
        raise ArithmeticError('Matrices do not have same dimension')
    result = [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    return result

def scalarMultiply(alpha,A):
    """
    Multiply every element of a matrix by a scalar number

    :param alpha: scalar
    :param A: Matrix (list of lists) [m x n]
    :return: alpha*A [m x n]
    """
    result = [[alpha*A[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    return result

def scalarDivide(alpha, A):
    """
    Divide every element of a matrix by a scalar number; raises arithmetic error of alpha is zero

    :param alpha: scalar (cannot be zero)
    :param A: Matrix (list of lists) [m x n]
    :return: A / alpha [m x n]
    """
    if math.isclose(alpha,0.0):
        raise ArithmeticError('Cannot divide by zero')
    result = [[A[i][j]/alpha for j in range(len(A[0]))] for i in range(len(A))]
    return result

def dotProduct(A,B):
    """
    Matrix inner product, raises arithmetic error if dimensions don't match

    :param A: matrix (list of lists) [m x n]
    :param B: matrix (list of lists) [m x n]
    :return: (A')*B [n x n]
    """
    if any([len(A)!=len(B),len(A[0])!=len(B[0])]):
        raise ArithmeticError('Vectors must have same dimension')
    result = multiply(transpose(A),B)
    return result

def skew(x,y,z):
    """
    Defines the skew symmetric matrix (also known as cross product matrix)

    :param x: (float) x-component of vector
    :param y: (float) y-component of vector
    :param z: (float) z-component of vector
    :return: [Ax] skew symmetric matrix [3 x 3]
    """
    result = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    return result

def crossProduct(A,B):
    """
    Vector cross product, raises arithmetic error if vectors are not [3 x 1]

    :param A: vector (list of lists) [3 x 1]
    :param B: vector (list of lists) [3 x 1]
    :return: A x B vector [3 x 1]
    """
    if any([len(A) != 3, len(A[0]) != 1,len(B) != 3, len(B[0]) != 1]):
        raise ArithmeticError('Cross Product only defined for 3x1 vectors')
    result = multiply(skew(A[0][0],A[1][0],A[2][0]),B)
    return result

def offset(A,x,y,z):
    """
    Shift each column of matrix A by the corresponding entry x,y,z; raises arithmetic error if the matrix A is not [3xn]

    :param A: Matrix to be shifted [3 x n]
    :param x: First column offset
    :param y: Second column offset
    :param z: Third column offset
    :return: Shifted matrix [3 x n]
    """
    if len(A[0]) != 3:
        raise ArithmeticError('Offset only works on [n x 3] matrices')
    result = [[pts[0] + x, pts[1] + y, pts[2] + z] for pts in A]
    return result

def vectorNorm(v):
    """
    Return a unit vector in the same direction as the input vector; raises arithmetic error if the vector v is not [nx1]

    :param v: [nx1] vector to be scaled
    :return: vbar = v/||v||, same dimensions as v
    """
    if len(v[0]) != 1:
        raise ArithmeticError('VectorNorm only works on [n x 1] vectors')
    normlist = [row[0] for row in v]
    norm = math.sqrt(sum(x**2 for x in normlist))
    return scalarDivide(norm,v)


def size(A):
    """
    Size of a matrix as [row, column]

    :param A: input matrix
    :return: list of [row, column]
    """
    return [len(A),len(A[0])]

def matrixPrint(A):
    """
    Prints the matrix in a tabbed column format to the output

    :param A: matrix (list of lists) to be printed
    :return: none
    """
    for A_row in A:
        print('\t'.join(['{: 7.3f}'.format(a) for a in A_row]))
    return




# Test Harness -- test the code with known good examples
# if __name__ == "__main__":
#     import MatrixMath
#     print('hello')
#     A = [[0,1,3],[1,0,5],[3,4,-3]]
#     print('A =')
#     MatrixMath.matrixPrint(A)
#     A = MatrixMath.transpose(A)
#     print('A transpose = ')
#     MatrixMath.matrixPrint(A)
#     B = [[2, 1, 6],[1, 2, -3],[-1,2, 7]]
#     print('B = ')
#     MatrixMath.matrixPrint(B)
#     C = MatrixMath.multiply(A,B)
#     print('A * B = ')
#     MatrixMath.matrixPrint(C)
#     C = MatrixMath.add(A,B)
#     print('A + B = ')
#     MatrixMath.matrixPrint(C)
#     alpha = 1.5
#     C = MatrixMath.scalarMultiply(alpha,A)
#     print(str(alpha)+' * A =')
#     MatrixMath.matrixPrint(C)
#     R = MatrixMath.transpose([[1,2,3]])
#     print('R = ')
#     MatrixMath.matrixPrint(R)
#     print('dot product of R =')
#     C = MatrixMath.dotProduct(R,R)
#     MatrixMath.matrixPrint(C)
#     C = MatrixMath.skew(R[0][0],R[1][0],R[2][0])
#     print('Skew symmetric of R =')
#     MatrixMath.matrixPrint(C)
#     B = [[-1],[-3],[-5]]
#     print('B = ')
#     MatrixMath.matrixPrint(B)
#     C = MatrixMath.crossProduct(R,B)
#     print('Cross Product of R and B =')
#     MatrixMath.matrixPrint(C)
#     print(MatrixMath.size(C))
#     A = [[0, 1, 3], [1, 0, 5], [3, 4, -3]]
#     Xo = [5,10,15]
#     print('A = ')
#     MatrixMath.matrixPrint(A)
#     print('Xo = ',Xo)
#     C = MatrixMath.offset(A,Xo[0],Xo[1],Xo[2])
#     print('A offset by Xo = ')
#     MatrixMath.matrixPrint(C)
