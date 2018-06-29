#ifndef MD5_H
#define MD5_H

typedef struct
{
  unsigned int count[2];
  unsigned int state[4];
  unsigned char buffer[64];   
} md5_ctx;//MD5_CTX

#define MD5_STRING_LEN (32)

#define F(x,y,z) ((x & y) | (~x & z))
#define G(x,y,z) ((x & z) | (y & ~z))
#define H(x,y,z) (x^y^z)
#define I(x,y,z) (y ^ (x | ~z))
#define ROTATE_LEFT(x,n) ((x << n) | (x >> (32-n)))

#define FF(a,b,c,d,x,s,ac) \
{ \
  a += F(b,c,d) + x + ac; \
  a = ROTATE_LEFT(a,s); \
  a += b; \
}
#define GG(a,b,c,d,x,s,ac) \
{ \
  a += G(b,c,d) + x + ac; \
  a = ROTATE_LEFT(a,s); \
  a += b; \
}
#define HH(a,b,c,d,x,s,ac) \
{ \
  a += H(b,c,d) + x + ac; \
  a = ROTATE_LEFT(a,s); \
  a += b; \
}
#define II(a,b,c,d,x,s,ac) \
{ \
  a += I(b,c,d) + x + ac; \
  a = ROTATE_LEFT(a,s); \
  a += b; \
}                                            

extern int compute_md5(unsigned char *data,int len,unsigned char *md5_value);
extern int md5_string_to_hex(unsigned char *md5,unsigned char *value);

#endif
