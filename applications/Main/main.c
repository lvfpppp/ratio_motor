
// #define EN_CALLBACK_TEST
#define EN_ADJUST_TEST

extern int callback_main(void);
extern int adjust_main(void);

int main(void)
{
#if defined(EN_CALLBACK_TEST)
    callback_main();
#elif defined(EN_ADJUST_TEST)
    adjust_main();
#endif
}
