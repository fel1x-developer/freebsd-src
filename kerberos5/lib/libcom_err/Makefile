
PACKAGE=	kerberos-lib

LIB=	com_err
SRCS=	com_err.c error.c
INCS=	com_err.h com_right.h
MAN=	com_err.3
CFLAGS+=	-I.
LDFLAGS=	-Wl,--no-undefined
VERSION_MAP=	${KRB5DIR}/lib/com_err/version-script.map

.include <bsd.lib.mk>

.PATH:	${KRB5DIR}/lib/com_err
