
PACKAGE=	kerberos

PROG=	kinit
CFLAGS+=-I${KRB5DIR}/lib/roken
LIBADD=	kafs5 krb5 gssapi heimntlm roken hcrypto vers

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kuser
