
PACKAGE=	kerberos-lib

LIB=	heimbase
LDFLAGS=	-Wl,--no-undefined
VERSION_MAP= ${KRB5DIR}/lib/base/version-script.map

INCS=	heimbase.h

SRCS= \
	array.c \
	base64.c \
	bool.c \
	bsearch.c \
	data.c \
	db.c \
	dict.c \
	dll.c \
	error.c \
	heimbase.c \
	json.c \
	null.c \
	number.c \
	string.c \
	test_base.c

CFLAGS+=	-I${KRB5DIR}/include

CLEANFILES=	base64.c

base64.c:
	rm -f base64.c
	ln -s ${KRB5DIR}/lib/roken/base64.c .

.include <bsd.lib.mk>

.PATH: ${KRB5DIR}/lib/base
