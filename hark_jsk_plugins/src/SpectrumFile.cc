#include <fstream>
#include <string.h>
#include <vector>
#include <Matrix.h>   
#include <math.h>
#include <SpectrumFile.h>
#include <myutils.h>
using namespace std;
namespace FD {
    SpectrumFile::SpectrumFile(){}
    SpectrumFile::SpectrumFile(string filename){/*{{{*/
        this->SetFilename(filename);
    }/*}}}*/

    void SpectrumFile::SetFilename(std::string filename){/*{{{*/
        filename_ = filename;
    }/*}}}*/

    void SpectrumFile::ClearFile(){/*{{{*/
        std::fstream fileio;
        fileio.open(filename_.c_str(),ios::out | ios::trunc);
        fileio.close();
    }/*}}}*/

    int SpectrumFile::GetMeanSpectrum(Matrix<float> &mat){/*{{{*/
        std::fstream fileio(filename_.c_str());
        if (!fileio) {
            return FAILURE;
        }
        int row=0,col=0;
        vector<string>::iterator itr;
        vector<string> floats;
        string buf;
        Matrix<float> mat_tmp;

        cout << ">>> Now loading prepared file:" << filename_ << "..." <<endl;
        //calc matrix's size/*{{{*/
        getline(fileio,buf);
        split(buf,' ',floats);
        for (itr = floats.begin(); itr != floats.end() ; itr++) {
            col++;
        }
        row++;
        while( !fileio.fail() ) {
            getline(fileio,buf);
            if( !fileio.eof() )
                row++;
        }
        nrows_ = row;
        ncols_ = col;
        mat.resize(1,ncols_);
        mat_tmp.resize(1,ncols_);
        for (int i = 0; i < ncols_; i++) {
            mat(0,i)=0.0;mat_tmp(0,i)=0.0;
        }
        fileio.close();

        // end calc matrix's size/*}}}*/

        fileio.open(filename_.c_str());
        row=0;col=0;
        while (!fileio.fail()) {
            getline(fileio,buf);
            split(buf,' ',floats);
            col=0;
            for (itr = floats.begin(); itr != floats.end() ; itr++) {
                mat_tmp(0,col) += (float)atof(itr->c_str());
                if(row%10==0){  //magic number
                    mat(0,col) += mat_tmp(0,col) / nrows_;
                    mat_tmp(0,col)=0.0;
                }
                col++;
            }
            row++;
        }
        cout << ">>> load-file done. ( '-^ )b" << endl;
        fileio.close();
    }/*}}}*/

    int SpectrumFile::GetLambdaD(Matrix<float> &mat){/*{{{*/
        std::fstream fileio(filename_.c_str());
        if (!fileio) {
            return FAILURE;
        }
        int row=0,col=0;
        vector<string>::iterator itr;
        vector<string> floats;
        string buf;
        Matrix<float> mat_tmp;

        cout << ">>> Now loading prepared file:" << filename_ << "..." <<endl;
        //calc matrix's size/*{{{*/
        getline(fileio,buf);
        split(buf,' ',floats);
        for (itr = floats.begin(); itr != floats.end() ; itr++) {
            col++;
        }
        row++;
        while( !fileio.fail() ) {
            getline(fileio,buf);
            if( !fileio.eof() )
                row++;
        }
        nrows_ = row;
        ncols_ = col;
        mat.resize(1,ncols_);
        mat_tmp.resize(1,ncols_);
        for (int i = 0; i < ncols_; i++) {
            mat(0,i)=0.0;mat_tmp(0,i)=0.0;
        }
        fileio.close();

        // end calc matrix's size/*}}}*/

        fileio.open(filename_.c_str());
        row=0;col=0;
        while (!fileio.fail()) {
            getline(fileio,buf);
            split(buf,' ',floats);
            col=0;
            for (itr = floats.begin(); itr != floats.end() ; itr++) {
                mat_tmp(0,col) += pow((float)atof(itr->c_str()),2);
                if(row%10==0){  //magic number
                    mat(0,col) += mat_tmp(0,col) / nrows_;
                    mat_tmp(0,col)=0.0;
                }
                col++;
            }
            row++;
        }
        cout << ">>> load-file done. ( '-^ )b" << endl;
        fileio.close();
    }/*}}}*/

    int SpectrumFile::SaveSpectrum(Matrix<float> &mat){/*{{{*/
        std::ofstream fileio;
        fileio.open(filename_.c_str(),std::ios::app);
        if (!fileio) {
            return FAILURE;
        }
        int nrows = mat.nrows();
        int ncols = mat.ncols();
        for (int row = 0; row < nrows; row++) {
            for (int col = 0; col < ncols; col++) {
                fileio << mat(row,col) << ' ';
            }
            fileio << std::endl;
        }
        fileio.close();
    }/*}}}*/

    void SpectrumFile::split(const std::string &str, char delim,std::vector<std::string> &output){/*{{{*/
        output.clear();
        std::istringstream iss(str);
        std::string tmp;
        while(getline(iss, tmp, delim)) 
            output.push_back(tmp);
    }/*}}}*/
} // namespace FD
