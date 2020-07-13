package org.vulcanrobotics.robotcorelib.math;

public class Matrix {

    private double[][] vals;
    public int rows;
    public int cols;

    public Matrix() {}

    public Matrix(double[][] vals) {
        this.vals = vals;
        rows = getRows();
        cols = getCols();
    }

    public void multiply(double multiplier) {
        for(int i = 0; i < cols; i++) {
            for(int j = 0; j < rows; j++) {
                double val = getVal(j + 1, i + 1) * multiplier;
                setVal(j + 1, i + 1, val);
            }
        }
    }

    public static Matrix multiply(Matrix multiplicand, Matrix multiplier) {
        try {
            if(!(multiplicand.cols == multiplier.rows)) {
                throw new MatrixOperationException("matrices row/column number must match");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }

        double[][] vals = new double[multiplicand.cols][multiplier.rows];
        Matrix product = new Matrix(vals);

        for(int i = 0; i < multiplier.rows; i++) {
            for(int j = 0; j < multiplicand.cols; j++) {
                double val = 0;
                for (int k = 0; k < multiplicand.rows; k++) {
                    val += multiplicand.getVal(k + 1, i + 1) * multiplier.getVal(j + 1, k + 1);
                }
                product.setVal(j + 1, i + 1, val);
            }
        }

        return product;
    }

    public static void multiply(Matrix in, double multiplier, Matrix out) {
        try {
            if(in.cols != out.cols || in.rows != out.rows) {
                throw new MatrixOperationException("in and out matrices must have the same dimensions");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }

        for(int i = 0; i < in.cols; i++) {
            for (int j = 0; j < in.rows; j++) {
                double val = in.getVal(j + 1, i + 1) * multiplier;
                out.setVal(j + 1, i + 1, val);
            }
        }
    }
    //TODO add non-static multiply matrix method

    public void transpose() {

        Matrix m = new Matrix(new double[cols][rows]);
        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < rows; j++) {
                double val = getVal(j + 1, i + 1);
                m.setVal(i + 1, j + 1, val);
            }
        }
        setMatrix(m);
    }

    public static void transpose(Matrix in, Matrix out) {
        try {
            if(in.cols != out.cols || in.rows != out.rows) {
                throw new MatrixOperationException("matrices must have the same dimensions!");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }
        for (int i = 0; i < in.cols; i++) {
            for (int j = 0; j < in.rows; j++) {
                double val = in.getVal(j + 1, i + 1);
                out.setVal(i + 1, j + 1, val);
            }
        }
    }

    public static Matrix transpose(Matrix in) {

        Matrix out = new Matrix(new double[in.cols][in.rows]);
        for (int i = 0; i < in.cols; i++) {
            for (int j = 0; j < in.rows; j++) {
                double val = in.getVal(j + 1, i + 1);
                out.setVal(i + 1, j + 1, val);
            }
        }
        return out;
    }

    public double det() {
        try {
            if(rows != cols) {
                throw new MatrixOperationException("matrix must be a square to find its determinant");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }
        double det = 0;

        if (rows == 1) {
            det = getVal(1, 1);
        }
        else if (rows == 2) {
            double ad = getVal(1, 1) * getVal(2, 2);
            double bc = getVal(2, 1) * getVal(1, 2);
            det = ad - bc;
        }
        else {

            Matrix sub = new Matrix(new double[cols - 1][rows - 1]);
            //sub array generation
            for (int j = 0; j < rows; j++) {
                for (int i = 1; i < rows; i++) {
                    int j1 = 0;
                    for (int n = 0; n < rows; n++) {
                        if (n == j)
                            continue;
                        double val = getVal(n + 1, i + 1);
                        sub.setVal(j1 + 1, i, val);
                        j1++;
                    }
                }
                //determinant of nxn matrix formula -
                det += Math.pow(-1.0, 1.0 + (j + 1)) * getVal(j + 1, 1) * sub.det();
            }
        }
        return det;
    }

    public Matrix inverse() {
        try {
            if(rows != cols) {
                throw new MatrixOperationException("matrix must be a square to find its inverse");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }
        Matrix minors = new Matrix(new double[cols][rows]);
        //sub array generation
        int i1 = 1;



        System.out.println(minors.toString());

        return minors;
    }

    public static Matrix removeRow(Matrix in, int row) {

        double[][] newVals = new double[in.cols][in.rows - 1];
        Matrix out = new Matrix(newVals);



        return out;
    }

    public static Matrix add(Matrix one, Matrix two) {
        //validate inputs
        try {
            if(one.cols != two.cols || one.rows != two.rows) {
                throw new MatrixOperationException("matrices row and column number must match");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }

        //initialize data
        double[][] vals = new double[one.cols][one.rows];
        Matrix sum = new Matrix(vals);

        //iterate through matrices
        for(int i = 0; i < one.cols; i++) {
            for(int j = 0; j < one.rows; j++) {
                double val = one.getVal(j + 1, i + 1) + two.getVal(j + 1, i + 1);
//                System.out.println(j);
                sum.setVal(j + 1, i + 1, val);
            }
        }

        return sum;
    }

    public static Matrix subtract(Matrix minuend, Matrix subtrahend) {
        //validate inputs
        try {
            if(minuend.cols != subtrahend.cols || minuend.rows != subtrahend.rows) {
                throw new MatrixOperationException("matrices row and column number must match");
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }

        //initialize data
        double[][] vals = new double[minuend.cols][minuend.rows];
        Matrix difference = new Matrix(vals);

        //iterate through matrices
        for(int i = 0; i < minuend.cols; i++) {
            for(int j = 0; j < minuend.rows; j++) {
                double val = minuend.getVal(j + 1, i + 1) - subtrahend.getVal(j + 1, i + 1);
                difference.setVal(j + 1, i + 1, val);
            }
        }

        return difference;
    }

    public double getVal(int row, int col) {
        return vals[col - 1][row - 1];
    }

    public void setVal(int row, int col, double value) {
        try {

            if(col > cols || row > rows) {
                System.out.println(col);
                System.out.println(cols);
                throw new MatrixOperationException("out of bounds of the matrix!");
            }

        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }

        vals[col - 1][row - 1] = value;
    }

    public int getCols() {
        return vals.length;
    }

    public int getRows() {
        int length = vals[0].length;
        try {
            for (double[] row : vals) {
                if(row.length != length) {
                    throw new MatrixOperationException("rows must all be the same length!");
                }
            }
        } catch (MatrixOperationException e) {
            e.printStackTrace();
        }

        return length;

    }

    public void setMatrix(Matrix m) {
        this.vals = m.vals;
        this.cols = m.cols;
        this.rows = m.rows;
    }

    public double toDouble() throws MatrixOperationException {
        if(rows != 1 || cols != 1) {
            throw new MatrixOperationException("Must be a 1x1 Matrix to convert to double.");
        }

        return getVal(1, 1);
    }

    public void isValid(double row, double col) throws MatrixOperationException {
        if(col > cols) {
            throw new MatrixOperationException("Cannot get value outside of matrix range.");
        }
        if(row > rows) {
            throw new MatrixOperationException("Cannot get value outside of matrix range.");
        }
    }

    public String toString() {
        StringBuilder s = new StringBuilder();
        for(int i = 0; i < cols; i++) {
            for(int j = 0; j < rows; j++) {
                s.append(this.getVal(j + 1, i + 1)).append(" | ");
            }
            s.append("\n");
        }

        return s.toString();
    }

}

class MatrixOperationException extends Exception {
    public MatrixOperationException() {
        super("Illegal Matrix Operation");
    }

    public MatrixOperationException(String s) {
        super(s);
    }
}
