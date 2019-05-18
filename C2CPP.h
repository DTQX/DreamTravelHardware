class C2CPP {
    public:
        C2CPP();

        static int readBit(int a);
};

#ifdef __cplusplus 
extern "C" {
#endif

int readBit_c(int a);

#ifdef __cplusplus 
}
#endif