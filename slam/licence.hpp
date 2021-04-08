
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#ifndef WIN32
#pragma GCC push_options 
#pragma GCC optimize ("O0") 

std::string GetCPU()
{
    char *szCpuId = new char[1024];
    //printf("%p\n",szCpuId);
    unsigned long s1,s2,s3,s4;
    //char string[128];
    char p1[128], p2[128];
    unsigned int eax = 0;
    unsigned int ebx,ecx,edx;

    asm volatile
            (   "cpuid"
                : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
                : "0"(0)
                );
    snprintf(szCpuId, 5, "%s", (char *)&ebx);
    snprintf(szCpuId+4, 5, "%s", (char *)&edx);
    snprintf(szCpuId+8, 5, "%s", (char *)&ecx);

    asm volatile
            (   "movl $0x01 , %%eax ; \n\t"
                "xorl %%edx , %%edx ;\n\t"
                "cpuid ;\n\t"
                "movl %%edx ,%0 ;\n\t"
                "movl %%eax ,%1 ; \n\t"
                :"=m"(s1),"=m"(s2)
                );

    sprintf((char *)p1, "-%08X-%08X-", s1, s2);
    snprintf(szCpuId+12, 20, "%s", (char *)p1);

    asm volatile
            (   "movl $0x03,%%eax ;\n\t"
                "xorl %%ecx,%%ecx ;\n\t"
                "xorl %%edx,%%edx ;\n\t"
                "cpuid ;\n\t"
                "movl %%edx,%0 ;\n\t"
                "movl %%ecx,%1 ;\n\t"
                :"=m"(s3),"=m"(s4)
                );

    sprintf((char *)p2, "%08X-%08X", s3, s4);
    snprintf(szCpuId+31, 19, "%s", (char *)p2);

    //printf("%p\n",szCpuId);
    std::string ret(szCpuId);
    //std::cout<<"AGV licence check!"<<std::endl;
    delete szCpuId;
    return ret;
}
using namespace std;

char MakecodeChar(char c,int key){
    return c=c^key;
}
void Makecode(char *pstr,int *pkey){
    int len=strlen(pstr);//获取长度
    for(int i=0;i<len;i++)
        *(pstr+i)=MakecodeChar(*(pstr+i),pkey[i%5]);
}

char CutcodeChar(char c,int key){
    return c^key;
}
void Cutecode(char *pstr,int *pkey){
    int len=strlen(pstr);
    for(int i=0;i<len;i++)
        *(pstr+i)=CutcodeChar(*(pstr+i),pkey[i%5]);
}


unsigned circleShift(const unsigned& word,const int& bits){
    return (word<<bits) | ((word)>>(32-bits));
}

unsigned sha1Fun(const unsigned& B,const unsigned& C,const unsigned& D,const unsigned& t){

    switch (t/20){
    case 0:     return (B & C) | ((~B) & D);
    case 2:     return (B & C) | (B & D) | (C & D);
    case 1:
    case 3:     return B ^ C ^ D;
    }

    return t;
}

string sha1(const string& strRaw){

    string str(strRaw);

    int tmp = 0;
    for( int i = 0; i < str.length(); i ++)
    {
        tmp += str[i] * cos(i);
        tmp = tmp%255;
        str[i] += tmp;
    } 

    return str;
}

string string2hex(string &buf)
{
    string ret;
    for( int i = 0; i < buf.length(); ++i)
    {
        char tmp[3];
        sprintf(tmp, "%02x", (unsigned char)buf[i]);
        ret += tmp;
    }
    return ret;
}
string hex2string(string &buf){
    string ret;
    for( int i = 0 ; i < buf.length() / 2 ; i ++)
    {
        string tmp = buf.substr(i*2, 2);
        int v;
        sscanf(tmp.c_str(),"%x", &v);
        ret += char(v);
    }
    return ret;
}

int licence_verify()
{ 
    
    int *key = new int[1024];
    std::string cpu = GetCPU();

    //encode cpu
    for(int i = 0; i < 1024; i++)
        key[i] = i*i + 1;
    Makecode(const_cast<char *> (cpu.c_str()), key);
    //std::cout<<"cpu: "<<cpu<<endl;

    //decode licence
    std::string lic = sha1(cpu);
    //std::cout<<"licence: "<< lic <<endl;
    for(int i = 0; i < 1024; i++)
        key[i] = i + cos(i) * i + 18;
    Cutecode(const_cast<char *> ( lic.c_str() ), key);
    delete key;
    string envlic;
    if(getenv("AGV_KEY") )
    {
        envlic = getenv("AGV_KEY");
    }
    std::cerr<<"mechine id: "<< string2hex(cpu) <<endl;
    std::cout<<"current key: "<<envlic<<endl;
    //std::cout<< lic <<endl;
    //std::cout<< hex2string( envlic ) <<endl;
    if(  lic == hex2string( envlic ))
        return 0;
    else
    {
        std::cerr<<"software not licenced! please contact volans.liao@gmail.com."<<std::endl;
    }

    return -1;
}
#ifdef VOLANSLIAO
void licence(string& cpu_)
{
    int *key = new int[1024];
    string cpu = hex2string(cpu_);
    //cerr<<"cpu: "<< cpu<< endl;

    //decode licence
    std::string lic = sha1(cpu);
    for(int i = 0; i < 1024; i++)
        key[i] = i + cos(i) * i + 18;
    //std::cout<<"licence: "<< lic <<endl;
    Makecode(const_cast<char *> ( lic.c_str() ), key);
    cerr<<"key: "<<string2hex(lic)<<endl;

    delete key;
}
int main(int argc, char **argv)
{
    string cpu;
    if(argc > 1)
        cpu = argv[1];
    licence(cpu);
    cout<<licence_verify()<<endl;
}
#endif

#pragma GCC pop_options 

#else
int licence_verify()
{
	return 0;
}
#endif