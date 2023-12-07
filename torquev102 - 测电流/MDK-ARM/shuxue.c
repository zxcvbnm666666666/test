#include <math.h>
#include <stdio.h>

double integral(double(*fun)(double x), double a, double b, int n)
{
    double sum,step,result;
    int i;
    sum=(fun(a)+fun(b))/2;
    step=(b-a)/n; /*????*/
    for(i=1;i<n;i++)
    sum=sum+fun(a+i*step);
    result=sum*step;
    return result;/*?????*/
}

double function(double x)
{
    return (x*sin(x));  /*????????????*/
}

  main()
{
    double result;
    result=integral(function,1.0,2.0,150);/*????????????????*/
    printf("result=%f\n",result);
}