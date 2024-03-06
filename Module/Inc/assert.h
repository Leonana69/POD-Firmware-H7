#ifndef __ASSERT_H__
#define __ASSERT_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

void assertFail(char *exp, char *file, int line);

#ifdef __cplusplus
}
#endif

#endif // __ASSERT_H__