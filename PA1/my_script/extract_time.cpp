#include<iostream>
#include<string>
#include<sstream>
using namespace std;

double extract_time(string str){
    char c;
    stringstream ss, ss2;
    double min = 0.0;
    double sec = 0.0;
    ss << str;
    
    while (ss >> c){
        if (c == 'm'){
            ss2 >> min;
            ss2.clear();
            ss2.str("");
        }
        else if (c == 's'){
            ss2 >> sec;
            ss2.clear();
            ss2.str("");
            break;
        }
        else{
            // c is digital numbers or '.'
            ss2 << c;
        }
    }
    
    return sec + min * 60.0;
}

int main(){
    string str;
    stringstream ss;
    double cputime = 0.0;
    double realtime = 0.0;
    while(getline(cin, str)){
        ss << str; //  ex "real 0m0.000s"
        ss >> str; // "real"
        if (str == "real"){
            ss >> str;
            realtime = extract_time(str);
            ss.clear();
            ss.str("");
        }
        else if (str == "user" || str == "sys"){
            ss >> str;
            cputime += extract_time(str);
            ss.clear();
            ss.str("");
        }
        else{
            ss.clear();
            ss.str("");
            continue;
        }
    }
    cout << "real " << realtime << endl;
    cout << "cpu " << cputime << endl;
    return 0;
}