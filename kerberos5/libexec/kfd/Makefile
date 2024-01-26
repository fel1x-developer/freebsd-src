
PACKAGE=	kerberos

PROG=	kfd
MAN=	kfd.8
SRCS=	kfd.c
CFLAGS+=	-I${KRB5DIR}/lib/asn1 \
		-I${KRB5DIR}/lib/roken
LIBADD=	asn1 crypt hcrypto krb5 roken vers

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/appl/kf
