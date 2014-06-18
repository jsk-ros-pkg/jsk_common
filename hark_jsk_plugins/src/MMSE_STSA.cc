#include <iostream>
#include <BufferedNode.h>
#include <Buffer.h>
#include <Vector.h>
#include <Matrix.h>
#include <Stream.h>
#include <math.h>
#include <algorithm>
#include <SpectrumFile.h>
#include <myutils.h>
#include <boost/math/special_functions/bessel.hpp>
#include <boost/math/special_functions/gamma.hpp>
#include <boost/math/special_functions/bessel.hpp>
#include <boost/math/special_functions/gamma.hpp>

using namespace std;
using namespace FD;
using namespace boost;
class MMSE_STSA;
DECLARE_NODE(MMSE_STSA);
/*Node
 *
 * @name MMSE_STSA
 * @category JSK
 * @description Signal Enhancer (Noise Canceller) using MMSE_STSA method.
 *
 * @input_name INPUT1
 * @input_type Matrix<complex<float> >
 * @input_description result of FFT
 *
 * @output_name OUTPUT
 * @output_type Matrix<complex<float> >
 * @output_description Noise-Cancelled audio
 *
 * @parameter_name LENGTH
 * @parameter_type int
 * @parameter_value 512
 * @parameter_description FFT's sample length [default: 512]
 *
 * @parameter_name SUBTRACTION_STYLE
 * @parameter_description choose whether you prepare wav before or take some first frames.
 * @parameter_type string
 * @parameter_value PREPARE
 * @parameter_list PREPARE:TAKEFIRST
 *
 * @parameter_name TAKEFIRST_FRAMES
 * @parameter_type int
 * @parameter_value 30
 * @parameter_valid SUBTRACTION_STYLE=TAKEFIRST
 * @parameter_description the number of noise sampling frames
 *
 * @parameter_name PREPARED_FILENAME
 * @parameter_type string
 * @parameter_value prepared_file
 * @parameter_valid SUBTRACTION_STYLE=PREPARE
 * @parameter_description prepared filename
 END*/
class MMSE_STSA : public BufferedNode {
    typedef Matrix<float> FMatrix;
    typedef Matrix<complex<float> > FCMatrix;
    int input1ID_;
    int outputID_;
    int length_;
    int pslength_;
    float takefirst_frames_;
    float alpha_;
    float gamma15_;
    Matrix<float> lambdaD_;
    Matrix<float> prevGamma_;
    SpectrumFile spectrum_file_;
    string subtraction_style_;

    public:
    MMSE_STSA(string nodeName, ParameterSet params) //{{{
        : BufferedNode(nodeName, params)
    {
        input1ID_ = addInput("INPUT1");
        outputID_ = addOutput("OUTPUT");
        length_ = dereference_cast<int>(parameters.get("LENGTH"));

        subtraction_style_  = object_cast<String>(parameters.get("SUBTRACTION_STYLE"));
        if (subtraction_style_=="PREPARE") {
            string prepared_filename = object_cast<String>(parameters.get("PREPARED_FILENAME"));
            spectrum_file_.SetFilename(prepared_filename);
            if(spectrum_file_.GetLambdaD(lambdaD_)==FAILURE)
                throw new NodeException(NULL,"MMSE_STSA:No prepared file exists.",__FILE__,__LINE__);
        }else if (subtraction_style_=="TAKEFIRST") {
            takefirst_frames_ = dereference_cast<int>(parameters.get("TAKEFIRST_FRAMES"));
        }

        pslength_ = length_/2 + 1;
        alpha_ = 0.98;
        gamma15_ = math::tgamma<float>(1.5);
        inOrder = true;
        cout << "MMSE_STSA initialized" << endl;
    }//}}}

    void calculate(int output_id, int count, Buffer &out)  //{{{
    {
        RCPtr<Matrix<complex<float > > > input1 = getInput(input1ID_, count);

        int nrows = input1->nrows(); //channel number
        int ncols = input1->ncols(); //spectrum

        if (ncols != pslength_) {
            throw new NodeException(NULL, string("MMSE_STSA: The FFT's LENGTH must be the same to LENGTH"), __FILE__, __LINE__);
        }

        RCPtr<Matrix<complex<float > > > output(new Matrix<complex<float > >(nrows,ncols));
        (*(outputs[outputID_].buffer))[count] = output;

        if(count == 0){
            prevGamma_.resize(nrows,ncols);
            for (int i = 0; i < nrows; i++) { for (int j = 0; j < ncols; j++) { prevGamma_(i,j)=0.0; } }
            if (subtraction_style_=="TAKEFIRST") {
                lambdaD_.resize(nrows,ncols);
                for (int i = 0; i < nrows; i++) { for (int j = 0; j < ncols; j++) { lambdaD_(i,j)=0.0; } }
            }
        }

        Matrix<float> Yr(nrows,ncols);
        Matrix<float> Yp(nrows,ncols);
        for (int row = 0; row < nrows; row++) {
            for (int col = 0; col < ncols; col++) {
                Yr(row,col) = abs((*input1)(row,col));
                Yp(row,col) = arg((*input1)(row,col));
            }
        }


        //最初の5countはノイズとして取得する
        if (subtraction_style_ == "TAKEFIRST" && count < takefirst_frames_) {
            for (int row = 0; row < nrows; row++) {
                for (int col = 0; col < pslength_; col++) {
                    lambdaD_(row,col) += pow(Yr(row,col),2);
                }
            }
        //その後ノイズ除去
        } else {
            if (subtraction_style_ == "TAKEFIRST" && count == takefirst_frames_) {
                for (int row = 0; row < nrows; row++) {
                    for (int col = 0; col < pslength_; col++) {
                        lambdaD_(row,col) = lambdaD_(row,col)/(takefirst_frames_*1.0);
                    }
                }
            }

            Matrix<float> Gamma(nrows,ncols);
            Matrix<float> xi(nrows,ncols);
            Matrix<float> nu(nrows,ncols);
            
            //cout << "count: "<< count <<  endl;
            // for each channel...
            for (int row = 0; row < nrows; row++) { 
                for (int col = 0; col < ncols; col++) {
                    Gamma(row,col) = pow(Yr(row,col),2) / lambdaD_(row,col);
                    //cout << Gamma(row,col) << "\t" << lambdaD_(row,col) << endl;
                }

                for (int col = 0; col < ncols; col++) {
                    xi(row,col) = alpha_ * pow(Gamma(row,col),2) * prevGamma_(row,col) + (1 - alpha_) * max<float>(Gamma(row,col) - 1.0, 0.0);
                    //cout << xi(row,col) << endl;
                }

                //ObjectRef(prevGamma_) = Gamma.clone();
                for (int col = 0; col < ncols; col++) {
                    prevGamma_(row,col) = Gamma(row,col);
                }

                for (int col = 0; col < ncols; col++) {
                    nu(row,col) = Gamma(row,col) * xi(row,col) / ( 1 + xi(row,col));
                    //cout << nu(row,col) << endl;
                }

                for (int col = 0; col < ncols; col++) {
                    
                    try {
                        Gamma(row,col) = (gamma15_ * sqrt(nu(row,col)) / Gamma(row,col)) * exp(-nu(row,col)/2) * ((1+nu(row,col)) * boost::math::cyl_bessel_i<int,float>(0,nu(row,col)/2) + nu(row,col) * boost::math::cyl_bessel_i<int,float>(1,nu(row,col)/2));
                    } catch (const std::exception& e) {
                        cerr << "MMSE_STSA: Gamma=(" << row << "," << col << ") " << Gamma(row,col) << e.what() << endl << nu(row,col) << endl;
                        // throw(e);
                    }
                    
                    // 事前 S/N 比の推定と 計算途中で G の NaN となった要素に対し、ウィナーフィルターで置き換えている。 
                    if(isnan(Gamma(row,col)) || isinf(Gamma(row,col || Gamma(row,col) > 1300 ))){
                        Gamma(row,col) = xi(row,col) / (xi(row,col) + 1.0);
                    }
                }
                for (int col = 0; col < ncols; col++) {
                    Yr(row,col) = Gamma(row,col) * Yr(row,col);
                    (*output)[row][col] = complex<float>(Yr(row,col),Yp(row,col));
                }
            }
        }

        /*****************************************************************/
        //from reference
        //output:
        // RCPtr<Matrix<float> > output1(new Matrix<float>(rows, cols));
        // (*(outputs[output1ID].buffer))[count] = output1;
        //input:
        // RCPtr<Matrix<float> > input1 = getInput(input1ID, count);
        // (使用時,(*input1)(i,j).Matrix<int> 型も同様
        //
        /*****************************************************************/

        // Main loop routine ends here.
    }//}}}
};

