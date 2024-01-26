
PACKAGE=	kerberos

PROG=	compile_et
MAN=
SRCS=	compile_et.c parse.y lex.l
LIBADD=	roken vers crypt
CFLAGS+=-I${KRB5DIR}/lib/com_err \
	-I${SRCTOP}/kerberos5/include

WARNS?=	0

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/lib/com_err
