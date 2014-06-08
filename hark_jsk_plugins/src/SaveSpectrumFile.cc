#include <iostream>
#include <BufferedNode.h>
#include <Buffer.h>
#include <Vector.h>
#include <Matrix.h>
#include <Stream.h>
#include <math.h>
#include <SpectrumFile.h>

using namespace std;
using namespace FD;
class SaveSpectrumFile;
DECLARE_NODE(SaveSpectrumFile);
/*Node
 *
 * @name SaveSpectrumFile
 * @category JSK
 * @description Save Spectrum to file
 *
 * @input_name INPUT1
 * @input_type Matrix<complex<float> >
 * @input_description result of FFT. This must be Matrix, not Map.
 *
 * @output_name OUTPUT
 * @output_type Matrix<complex<float> >
 * @output_description same as input1.
 *
 * @parameter_name FILENAME
 * @parameter_type string
 * @parameter_value prepared_file
 * @parameter_description the file name to save matrix.
 END*/
class SaveSpectrumFile : public BufferedNode {
    typedef Matrix<float> FMatrix;
    typedef Matrix<complex<float> > FCMatrix;
    int input1ID_;
    int outputID_;
    string filename_;
    SpectrumFile spcfile_;

    public:
    SaveSpectrumFile(string nodeName, ParameterSet params) //{{{
        : BufferedNode(nodeName, params)
    {
        input1ID_ = addInput("INPUT1");
        outputID_ = addOutput("OUTPUT");
        filename_ = object_cast<String>(parameters.get("FILENAME"));
        spcfile_.SetFilename(filename_);
        spcfile_.ClearFile();
        inOrder = true;
        cout << "SaveSpectrumFile initialized" << endl;
    }//}}}

    void calculate(int output_id, int count, Buffer &out)  //{{{
    {
        RCPtr<Matrix<complex<float > > > input1 = getInput(input1ID_, count);
        out[count]=input1;

        int nrows = input1->nrows(); //channel number
        int ncols = input1->ncols(); //spectrum

        Matrix<float> Yr(nrows,ncols);
        for (int row = 0; row < nrows; row++) {
            for (int col = 0; col < ncols; col++) {
                Yr(row,col) = abs((*input1)(row,col));
            }
        }

        spcfile_.SaveSpectrum(Yr);

        // Main loop routine ends here.
    }//}}}
};

