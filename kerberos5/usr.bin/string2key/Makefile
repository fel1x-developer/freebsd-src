
PACKAGE=	kerberos

PROG=	string2key
MAN=	string2key.8
CFLAGS+=	-I${KRB5DIR}/kdc \
		-I${KRB5DIR}/lib/asn1 \
		-I${KRB5DIR}/lib/krb5 \
		-I${KRB5DIR}/lib/roken \
		-I${KRB5DIR}/lib/windc \
		-I${SRCTOP}/contrib/com_err
SRC=	string2key.c \
	headers.h
LIBADD=	hdb krb5 hcrypto asn1 roken vers

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kdc
