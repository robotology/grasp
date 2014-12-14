# Copyright (C) 2013 King's College London
# Authors: Kris De Meyer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

#define the name of the message files (.h/.xml/.xsd/.xslt)
set(MsgFileName GraspMessages)
set(VocabFileName VocabDefinitions)
set(LocName "${CMAKE_CURRENT_SOURCE_DIR}/${MsgFileName}")

#add a target that will not generate a file
add_custom_target(MessageFormats 
	SOURCES ${MsgFileName}.xml ${MsgFileName}.h ${VocabFileName}.h)

#on Windows, add functionality to generate the C++ code from XML using JScript
#this step will run automatically in VS when building the project
if(WIN32)
add_custom_command(OUTPUT ${LocName}.h
		   COMMAND cscript generate-code.js ${MsgFileName}.xml ${MsgFileName}.xslt ${MsgFileName}.h
		   DEPENDS ${LocName}.xml
		   COMMENT "Generating C++ code from ${MsgFileName}.xml")
endif(WIN32)

#on Linux, use xsltproc, a command-line XSLT processor
if (UNIX)
add_custom_command(OUTPUT ${LocName}.h
                   COMMAND xsltproc -o "${LocName}.h" "${LocName}.xslt" "${LocName}.xml"
                   DEPENDS ${LocName}.xml
                   COMMENT "Generating C++ code from ${MsgFileName}.xml")
endif(UNIX)

#make this target depend on the TypeSafeBottle target
add_dependencies(MessageFormats TypeSafeBottle)

#add files to the correct folders in VS
source_group("Header Files" FILES ${MsgFileName}.h ${VocabFileName}.h)
source_group("Source Files" FILES ${MsgFileName}.xml)


#on Windows, we're generating a jscript in the current build directory that will
#automatically generate the header file from the xml file on build using the 
#custom command that was defined above
if(WIN32)
	file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/generate-code.js" "
	var args = WScript.Arguments;
	var xml = args(0);
	var xsl = args(1);
	var out = args(2);
	var xmldoc = new ActiveXObject(\"MSXML2.DOMDocument.6.0\");
	var xsldoc = new ActiveXObject(\"MSXML2.DOMDOCUMENT.6.0\");
	xmldoc.async = false;
	xmldoc.validateOnParse = false;
	xmldoc.load(\"${CMAKE_CURRENT_SOURCE_DIR}/\" + xml);
	if (xmldoc.parseError.errorCode != 0)
		WScript.Echo (\"XML Parse Error : \" + xmldoc.parseError.reason);

	xsldoc.async = false;
	xsldoc.resolveExternals = true;
	xsldoc.load(\"${CMAKE_CURRENT_SOURCE_DIR}/\" + xsl);
	if (xsldoc.parseError.errorCode != 0)
		WScript.Echo (\"XML Parse Error : \" + xsldoc.parseError.reason);

	/* Create a binary IStream */
	var outdoc = new ActiveXObject(\"ADODB.Stream\");
	outdoc.type = 1;
	outdoc.open();
	xmldoc.transformNodeToObject(xsldoc,outdoc);
	outdoc.SaveToFile(\"${CMAKE_CURRENT_SOURCE_DIR}/\" + out,2);
	")
endif(WIN32)



