/*
This is an example of how to use this modified MD5 library.  It provides
two easy-to-use methods, one for generating the MD5 hash, and the second
one to generate the hex encoding of the hash, which is frequently used.
*/

#include <MD5.h>

void setup()
{
  //initialize serial
  Serial.begin(9600);
  //give it a second
  delay(1000);
  // allocate an MD5_CTX structure
  MD5_CTX context;
  //generate the MD5 hash for our string
  unsigned char* hash=MD5::make_hash(&context, "hello world");
  //generate the digest (hex encoding) of our hash
  // make_digest() knows where the internal hash it stored
  // and always makes a digest of length 32
  char *md5str = MD5::make_digest(&context);
  //print it on our serial monitor
  Serial.println(md5str);
}

void loop()
{
}
