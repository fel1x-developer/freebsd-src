
PACKAGE=	kerberos

PROG=	kf
MAN=	kf.1
SRCS=	kf.c
CFLAGS+=	-I${KRB5DIR}/lib/asn1 \
		-I${KRB5DIR}/lib/roken
LIBADD=	krb5 asn1 roken vers

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/appl/kf
