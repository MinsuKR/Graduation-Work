#include <stdio.h>

void tpp1()
{
    printf("Hellow world");
}

int tpp(int a)
{
    a += 1;
    return a; 
}

int main(void)
{
    printf("%d",tpp(3));
    tpp1();
}