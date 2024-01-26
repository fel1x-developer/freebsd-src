
PACKAGE=	kerberos

PROG=	ksu
.if defined(ENABLE_SUID_K5SU)
BINMODE=4555
PRECIOUSPROG=
.endif
MAN=
SRCS=	su.c
CFLAGS+=-I${KRB5DIR}/lib/roken
LIBADD=	crypt kafs5 krb5 asn1 roken hcrypto vers

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/appl/su
