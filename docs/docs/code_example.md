An example

=== "Python"

    ~~~py title="isso.py"  linenums="69" hl_lines="3"
    import solution

    solution.do()
    ~~~

=== "C"

    ~~~c title="isso.c"  linenums="69"
    #include <stdio.h>

    int main() {
        void* a;
        *a;
        // segfault :)
    } 
    ~~~
