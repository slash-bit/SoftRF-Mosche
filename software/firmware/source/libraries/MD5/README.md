This is an MD5 library for the Arduino, based on scottmac's MD5 library, which you can find here:
https://github.com/scottmac/arduino

I created this because I was having a really hard time finding an easy-to-install and use libray for the Arduino,
so I decided to make my own. There is an example on how to use it.

### Installation
Create a folder named _MD5_ in the _libraries_ folder inside your Arduino sketch folder. If the
libraries folder doesn't exist, create it. Then copy everything inside. (re)launch the Arduino IDE.

You're done. Time for a mojito

### Usage

If you create md5 Hashes in a loop you must give the Memory back to the System 
```
unsigned char* hash=MD5::make_hash("hello world");
//generate the digest (hex encoding) of our hash
char *md5str = MD5::make_digest(hash, 16);
//print it on our serial monitor
Serial.println(md5str);
//Give the Memory back to the System if you run the md5 Hash generation in a loop
free(md5str);
//free dynamically allocated 16 byte hash from make_hash()
free(hash);
```

### Usage for this modified version

All use of malloc() has been removed.  An MD5_CTX context structure must be allocated before calling any functions in the library.  The pointers returned by make_hash() and make_digest() point to buffers within the MD5_CTX context structure.  Do not free() anything!  Usage:
```
MD5_CTX context;
unsigned char* hash=MD5::make_hash(context, "hello world");
char *md5str = MD5::make_digest(context);
   // make_digest() knows where the internal hash it stored
   // and always makes a digest of length 32
Serial.println(md5str);
```
Lower-level usage, allows adding the content in pieces:
```
MD5_CTX context;
MD5::MD5Init(&context);   // or MD5Initialize(&context, a, b, c, d); for custom keys
MD5::MD5Update(&context, arg_1, size_1);
...
MD5::MD5Update(&context, arg_n, size_n);
MD5::MD5Final(&context);
MD5::make_digest(&context);
```

