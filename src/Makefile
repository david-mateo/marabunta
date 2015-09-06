BASE=	BaseRobot.pyc	\
		MockBody.pyc	MockNetwork.pyc	\
		XBeeNetwork.pyc	ebotBody.pyc

# Models of robot behavior:
ROBOTS=	PerimeterDefenseRobot.pyc	HeadingConsensusRobot.pyc	MarchingRobot.pyc

OBJS=${BASE} $(addprefix robot_models/, ${ROBOTS})

LIBPATH=$(shell python -m site --user-site)

all:	${OBJS}

%.pyc:	%.py
	python -m py_compile $<

install:	${OBJS}
	mkdir -p ${LIBPATH}
	cp  ${OBJS} ${LIBPATH}

uninstall:
	rm $(addprefix ${LIBPATH}/,$(notdir ${OBJS}) )

update:
	git pull
	make install

clean:
	rm *.pyc robot_models/*.pyc
