
PACKAGE=	kerberos

PROG=	kimpersonate
MAN=	kimpersonate.8
CFLAGS+=	-I${KRB5DIR}/lib/hx509 \
		-I${KRB5DIR}/lib/asn1 \
		-I${KRB5DIR}/lib/roken \
		-I${KRB5DIR}/lib/sl -I.
LIBADD=	asn1 crypt gssapi hcrypto heimntlm kafs5 krb5 roken vers

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kuser
