#include <iostream>
#include "lights_recognize.hpp"
#include "accuracy.hpp"
#include "svm_train.hpp"
#include "test.hpp"
#include <stdio.h>

namespace traffic{
    namespace demo{
        int accuracyTestMain()
        {
			/*
			uint8_t * buf = new uint8_t[100];
			FILE *fp = popen("rospack find opencvExercise"，"r");
			fread(buf,1,100,fp);
			pclose(fp);
			
			std::string path = *buf;
			delete [] buf;
			
			std::cout << "path:" path <<std::endl;
			*/
            bool isExit = false;
            while(!isExit)
            {
                std::cout << "........accuracy ........"<< endl;
                int select = -1;
                bool isRepeat = true;

                while(isRepeat)
                {
                   
                    std::cin >> select;
                    bool isRepeat = false;
                    switch(select)
                    {
                        case 1:
                            accuracyTest("/resourses");
                            break;
                        case 2:
                            cout << "SVM Train" << endl;
                            {
                                traffic::SvmTrain svm("svm","xmldata/module/svm.xml");    
                                svm.train();
                            }
                            break; 
                        case 3:
                            cout << "SVM Test" << endl;
                            test("svm","xmldata/module/svm.xml");
                            break;
                        default:
                            std::cout << "input error " << ":";
                            isRepeat = true;
                            break;
                    }
                }
            }
            return 0;
        }
    } // namespace demo
}   //namesapce traffic


int main(int argc,const char*argv[])
{   
    bool isExit = false;
    while(!isExit)
    {
        int select = -1;
        bool isRepeat = true;
        while(isRepeat)
        {   
            cout << "please input your select: " << endl;
            std::cin >> select;
            isRepeat = false;
            switch(select)
            {
                case 1:
                    traffic::demo::accuracyTestMain();
                    break;
                default:
                    cout << "input_error" << ":";
                    isRepeat = true;
                    break; 
            }
        }
    }
    cout << "soble" << endl;
    return 0;
}
