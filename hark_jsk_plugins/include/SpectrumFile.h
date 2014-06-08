#ifndef __SPECTRUM_FILE__
#define __SPECTRUM_FILE__

#include <Matrix.h>
//streamから流れてくるスペクトルのAbsをファイルに保存するためのインターフェースとなるクラス
//file形式は
//            ←周波数→
//   ↑     (               )
// 時系列  (  floatの行列  )
//   ↓     (               )
namespace FD{
class SpectrumFile {
    public:
        std::string filename_;
        int nrows_;
        int ncols_;

        SpectrumFile();
        SpectrumFile(std::string);
        void SetFilename(std::string);
        /* fileを空する */
        void ClearFile();
        /* fileからmatを作る 
         * matrixが大きすぎて読みこめないときがあるのでやめた*/
        //int LoadSpectrum(Matrix<float>&);
        /* fileからmatを作り、時刻で平均する
         * matは1行n列のはず */
        int GetMeanSpectrum(Matrix<float>&);
        int GetLambdaD(Matrix<float>&);
        /* matをfileに追加・保存する
         * matは1行n列のはず */
        int SaveSpectrum(Matrix<float>&);
    private:
        void split(const std::string &,char,std::vector<std::string>&);
};

} // namespace FD
#endif // __SPECTRUM_FILE__
