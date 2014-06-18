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
#include "Map.h"

using namespace std;
using namespace FD;
using namespace boost;
class SpectrumSubtraction;
DECLARE_NODE(SpectrumSubtraction);
/*Node
 *
 * @name SpectrumSubtraction
 * @category JSK
 * @description Signal Enhancer (Noise Canceller) using SpectrumSubtraction method.
 *
 * @input_name INPUT1
 * @input_type any
 * @input_description result of FFT. Matrix<complex<float> > or Map<int,ObjectRef>(key:source ID/value:a vector of audio signals(Vector<complex<float> >).
 *
 * @output_name OUTPUT
 * @output_type any
 * @output_description Noise-Cancelled audio. the same type as input.
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
 *
 * @parameter_name METHOD
 * @parameter_type string
 * @parameter_value ABSOLUTE
 * @parameter_list  ABSOLUTE:SQUARE
 * @parameter_description subtraction method
 END*/
enum SUBTRACTION_METHOD {
  SQUARE,ABSOLUTE
};
class SpectrumSubtraction : public BufferedNode {
    typedef Matrix<float> FMatrix;
    typedef Matrix<complex<float> > FCMatrix;
    int input1ID_;
    int outputID_;
    int length_;
    int pslength_;
    float first_frames_;
    string subtraction_method_;
    string subtraction_style_;
    int subtraction_method_type_;
    SpectrumFile spectrum_file_;
    Matrix<float> lambdaD_;

    public:
    SpectrumSubtraction(string nodeName, ParameterSet params) //{{{
        : BufferedNode(nodeName, params)
    {
        input1ID_ = addInput("INPUT1");
        outputID_ = addOutput("OUTPUT");
        length_ = dereference_cast<int>(parameters.get("LENGTH"));
        subtraction_method_ = object_cast<String>(parameters.get("METHOD"));

        subtraction_style_  = object_cast<String>(parameters.get("SUBTRACTION_STYLE"));
        if (subtraction_style_=="PREPARE") {
            string prepared_filename = object_cast<String>(parameters.get("PREPARED_FILENAME"));
            spectrum_file_.SetFilename(prepared_filename);
            if(spectrum_file_.GetMeanSpectrum(lambdaD_)==FAILURE)
                throw new NodeException(NULL,"SpectrumSubtraction:No prepared file exists.",__FILE__,__LINE__);
        }else if (subtraction_style_=="TAKEFIRST") {
            first_frames_ = dereference_cast<int>(parameters.get("TAKEFIRST_FRAMES"));
        }

        if (subtraction_method_ == "SQUARE") {
            subtraction_method_type_=SQUARE;
        }else if(subtraction_method_=="ABSOLUTE"){
            subtraction_method_type_=ABSOLUTE;
        }else {
            throw new NodeException(NULL,"SpectrumSubtraction:No such method.",__FILE__,__LINE__);
        }

        pslength_ = length_/2 + 1;
        inOrder = true;
        cout << "SpectrumSubtraction initialized" << endl;
    }//}}}

complex<float> calc_out(float Yp,float Yr,float lamb){/*{{{*/
    float diff = Yr - lamb;
    float diff_2 = sqrt(pow(Yr,2)-pow(lamb,2));
    if (diff > 0) {
        switch (subtraction_method_type_) {
            case SQUARE:
                return complex<float>(diff_2,Yp);
            case ABSOLUTE:
                return complex<float>(diff,Yp);
        }
    } else {
        return complex<float>(0.0,Yp);
    }
}/*}}}*/

void calculate(int output_id, int count, Buffer &out)  //{{{
{
    ObjectRef in = getInput(input1ID_, count);

    if (typeid(*in) == typeid(Matrix<complex<float > >)) {/*{{{*/
        RCPtr<Matrix<complex<float > > > input1 = getInput(input1ID_, count);

        int nrows = input1->nrows(); //channel number
        int ncols = input1->ncols(); //spectrum

        RCPtr<Matrix<complex<float > > > output(new Matrix<complex<float > >(nrows,ncols));
        (*(outputs[outputID_].buffer))[count] = output;

        if(count == 0){
            lambdaD_.resize(nrows,ncols); //←resizeしないといけないのでconstructorには書けない
            for (int i = 0; i < nrows; i++) {
                for (int j = 0; j < ncols; j++) {
                    lambdaD_(i,j)=0.0;
                }
            }
        }

        if (ncols != pslength_) {
            throw new NodeException(NULL, string("SpectrumSubtraction: The FFT's LENGTH must be the same to LENGTH"), __FILE__, __LINE__);
        }

        Matrix<float> Yr(nrows,ncols);
        Matrix<float> Yp(nrows,ncols);
        for (int row = 0; row < nrows; row++) {
            for (int col = 0; col < ncols; col++) {
                Yr(row,col) = abs((*input1)(row,col));
                Yp(row,col) = arg((*input1)(row,col));
            }
        }


        if (subtraction_style_ == "TAKEFIRST") {
            //最初のfirst_frames_カウントはノイズとして取得する
            if (count < first_frames_) {
                for (int row = 0; row < nrows; row++) {
                    for (int col = 0; col < pslength_; col++) {
                        lambdaD_(row,col) += Yr(row,col);
                    }
                }
                //その後ノイズ除去の準備
            } else if (count == first_frames_) {
                for (int row = 0; row < nrows; row++) {
                    for (int col = 0; col < pslength_; col++) {
                        lambdaD_(row,col) = lambdaD_(row,col)/(first_frames_ * 1.0);
                    }
                }
            }
        }


        // for each channel...
        for (int row = 0; row < nrows; row++) { 
            for (int col = 0; col < ncols; col++) {
                (*output)[row][col] = calc_out(Yr(row,col),Yp(row,col),lambdaD_(row,col));
            }
        }
        /*}}}*/
    } else if (typeid(*in) == typeid(Map<int,ObjectRef>)) { /*{{{*/
        RCPtr<Map<int,ObjectRef> > input1 = getInput(input1ID_, count);
        RCPtr<Map<int,ObjectRef> > output(new Map<int, ObjectRef>);
        Map<int,ObjectRef>::iterator it = input1->begin();
        (*(outputs[outputID_].buffer))[count] = output;

        int nrows = input1->size();  //source number
        int ncols = (object_cast<Vector<complex<float> > >(it->second)).size(); //spectrum
        RCPtr<Vector<complex<float> > > out_tmp(new Vector<complex<float> >(ncols, 0.0f));

        if(count == 0){
            lambdaD_.resize(1,ncols); //←resizeしないといけないのでconstructorには書けない
            for (int i = 0; i < ncols; i++) {
                    lambdaD_(1,ncols)=0.0;
            }
        }

        if (ncols != pslength_) {
            throw new NodeException(NULL, string("SpectrumSubtraction: The FFT's LENGTH must be the same to LENGTH"), __FILE__, __LINE__);
        }

        Matrix<float> Yr(nrows,ncols);
        Matrix<float> Yp(nrows,ncols);
        it = input1->begin();
        for(int row = 0;it != input1->end();it++,row++){
            for (int col = 0; col < ncols; col++) {
                Vector<complex<float> > val = object_cast<Vector<complex<float> > >(it->second);
                Yr(row,col) = abs(val.at(col));
                Yp(row,col) = arg(val.at(col));
            }

            if (subtraction_style_ == "TAKEFIRST") {
                //最初のfirst_frames_カウントはノイズとして取得する
                if (count < first_frames_) {
                    for (int col = 0; col < pslength_; col++) {
                        lambdaD_(1,col) += Yr(row,col);
                    }
                    //その後ノイズ除去の準備
                } else if (count == first_frames_) {
                    for (int col = 0; col < pslength_; col++) {
                        lambdaD_(1,col) = lambdaD_(row,col)/(first_frames_ * 1.0);
                    }
                }
            }
        }

        it = input1->begin();
        for (int row = 0;it != input1->end() ;it++, row++) { 
            for (int col = 0; col < ncols; col++) {
                 (*out_tmp)[col] = calc_out(Yr(row,col),Yp(row,col),lambdaD_(1,col));
            }
            (*output)[it->first] = out_tmp;
        }
        /*}}}*/
    } else {
        throw new NodeException(NULL,"SpectrumSubtraction:unknown type input!",__FILE__,__LINE__);
    }

    /*********************** quote from reference ********************/ //{{{
    //output:
    // RCPtr<Matrix<float> > output1(new Matrix<float>(rows, cols));
    // (*(outputs[output1ID].buffer))[count] = output1;
    //input:
    // RCPtr<Matrix<float> > input1 = getInput(input1ID, count);
    // (使用時,(*input1)(i,j).Matrix<int> 型も同様
    //
    /*****************************************************************/ //}}}

    // Main loop routine ends here.
}//}}}
};

