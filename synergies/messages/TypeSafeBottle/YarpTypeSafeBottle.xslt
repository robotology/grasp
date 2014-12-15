<?xml version="1.0" encoding="UTF-8"?>
<!-- Copyright: (C) 2013 King's College London -->
<!-- Authors: Kris De Meyer -->
<!-- CopyPolicy: Released under the terms of the LGPLv2.1 or later -->
<xsl:transform version="1.0" xmlns:n1="http://darwin-project.eu/namespaces/TypeSafeBottle" xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:xs="http://www.w3.org/2001/XMLSchema" xmlns:fn="http://www.w3.org/2005/xpath-functions">
	<xsl:output method="text" encoding="UTF-8" indent="no"/>
	<xsl:template match="/n1:Bottles">
		<xsl:call-template name="file-preamble"/>
		<xsl:apply-templates select="n1:Namespace" mode="file-begin">
			<xsl:with-param name="indent" select="''"/>
		</xsl:apply-templates>
		<xsl:text>&#10;//pre-declaration of classes derived from StructBottle&#10;</xsl:text>
		<xsl:apply-templates select="n1:Struct" mode="predeclaration"/>
		<xsl:text>&#10;//pre-declaration of classes derived from WrapperBottle&#10;</xsl:text>
		<xsl:apply-templates select="n1:Wrapper" mode="predeclaration"/>
		<xsl:text>&#10;//named VectorBottles - implemented as typedefs&#10;</xsl:text>
		<xsl:apply-templates select="n1:Vector" mode="declaration"/>
		<xsl:text>&#10;//class declarations derived from StructBottle&#10;</xsl:text>
		<xsl:apply-templates select="n1:Struct" mode="declaration"/>
		<xsl:text>&#10;//class declarations derived from WrapperBottle&#10;</xsl:text>
		<xsl:apply-templates select="n1:Wrapper" mode="declaration"/>
		<xsl:text>&#10;//Function definitions&#10;</xsl:text>
		<xsl:apply-templates select="n1:Struct" mode="definition"/>
		<xsl:apply-templates select="n1:Wrapper" mode="definition"/>
		<xsl:apply-templates mode="file-end"/>
	</xsl:template>
	<xsl:template name="file-preamble">
		<xsl:text>//C++ code automatically generated from YarpTypeSafeBottle XML file - do not alter</xsl:text>
		<xsl:value-of select="concat($newline,$newline,'#pragma once',$newline,$newline)"/>
		<xsl:value-of select="concat('#include &quot;TypeSafeBottle/TypeSafeBottle.h&quot;',$newline,$newline)"/>
		<xsl:value-of select="concat('using namespace darwin::msg',$seminew,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Namespace" mode="file-begin">
		<xsl:param name="indent"/>
		<xsl:value-of select="concat($indent,'namespace ',@name,' {',$newline)"/>
		<xsl:apply-templates select="n1:Namespace" mode="file-begin">
			<xsl:with-param name="indent" select="concat($indent,$tab)"/>
		</xsl:apply-templates>
	</xsl:template>
	<xsl:template match="n1:Struct" mode="predeclaration">
		<xsl:value-of select="concat('class ',@className,$seminew)"/>
	</xsl:template>
	<xsl:template match="n1:Wrapper" mode="predeclaration">
		<xsl:value-of select="concat('class ',@className,$seminew)"/>
	</xsl:template>
	<xsl:template match="n1:Vector" mode="declaration">
		<xsl:value-of select="'typedef '"/>
		<xsl:call-template name="vectorbottle-type">
			<xsl:with-param name="type" select="@valueType"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' ',@className,$seminew)"/>
	</xsl:template>
	<xsl:template match="n1:Struct" mode="declaration">
		<xsl:value-of select="concat('class ',@className,' : public StructBottle {',$newline,'public:',$newline)"/>
		<xsl:value-of select="concat($tab,@className,'()',$seminew)"/>
		<xsl:apply-templates select="n1:Variable|n1:Vector" mode="declaration-get"/>
		<xsl:apply-templates select="n1:Variable[contains($builtintypes,@type)]" mode="declaration-set"/>
		<xsl:value-of select="concat('};',$newline,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Wrapper" mode="declaration">
		<xsl:value-of select="concat('class ',@className,' : public WrapperBottle {',$newline,'public:',$newline)"/>
		<xsl:value-of select="concat($tab,@className,'()',$seminew)"/>
		<xsl:apply-templates select="n1:Bottle" mode="declaration-get"/>
		<xsl:apply-templates select="n1:Bottle" mode="declaration-set"/>
		<xsl:apply-templates select="n1:Bottle" mode="declaration-subid"/>
		<xsl:value-of select="concat('};',$newline,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Struct" mode="definition">
		<xsl:call-template name="comment-implementations">
			<xsl:with-param name="className" select="@className"/>
		</xsl:call-template>
		<xsl:apply-templates select="." mode="definition-constructor"/>
		<xsl:apply-templates select="n1:Variable|n1:Vector" mode="definition-get"/>
		<xsl:apply-templates select="n1:Variable[contains($builtintypes,@type)]" mode="definition-set"/>
	</xsl:template>
	<xsl:template match="n1:Wrapper" mode="definition">
		<xsl:call-template name="comment-implementations">
			<xsl:with-param name="className" select="@className"/>
		</xsl:call-template>
		<xsl:apply-templates select="." mode="definition-constructor"/>
		<xsl:apply-templates select="n1:Bottle" mode="definition-get"/>
		<xsl:apply-templates select="n1:Bottle" mode="definition-set"/>
	</xsl:template>
	<xsl:template match="n1:Namespace" mode="file-end">
		<xsl:apply-templates select="n1:Namespace" mode="file-end"/>
		<xsl:variable name="name" select="@name"/>
		<xsl:value-of select="concat('}',' //end namespace ',$name,$newline)"/>
	</xsl:template>
	<!--declaration templates for functions-->
	<xsl:template match="n1:Variable" mode="declaration-get">
		<xsl:value-of select="$tab"/>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@name"/>
			<xsl:with-param name="returntype" select="@type"/>
		</xsl:call-template>
		<xsl:value-of select="$seminew"/>
	</xsl:template>
	<xsl:template match="n1:Variable" mode="declaration-set">
		<xsl:value-of select="$tab"/>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="concat('set',@name)"/>
			<xsl:with-param name="returntype" select="../@className"/>
			<xsl:with-param name="argtype" select="@type"/>
		</xsl:call-template>
		<xsl:value-of select="$seminew"/>
	</xsl:template>
	<xsl:template match="n1:Vector" mode="declaration-get">
		<xsl:variable name="vector">
			<xsl:call-template name="vectorbottle-type">
			<xsl:with-param name="type" select="@valueType"/>
		</xsl:call-template>
		</xsl:variable>
		<xsl:value-of select="$tab"/>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@name"/>
			<xsl:with-param name="returntype" select="$vector"/>
		</xsl:call-template>
		<xsl:value-of select="$seminew"/>
	</xsl:template>
	<xsl:template match="n1:Bottle" mode="declaration-get">
		<xsl:value-of select="$tab"/>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@subID"/>
			<xsl:with-param name="returntype" select="@valueType"/>
		</xsl:call-template>
		<xsl:value-of select="$seminew"/>
	</xsl:template>
	<xsl:template match="n1:Bottle" mode="declaration-set">
		<xsl:value-of select="$tab"/>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@subID"/>
			<xsl:with-param name="returntype" select="'void'"/>
			<xsl:with-param name="argtype" select="@valueType"/>
		</xsl:call-template>
		<xsl:value-of select="$seminew"/>
	</xsl:template>
	<xsl:template match="n1:Bottle" mode="declaration-subid">
		<xsl:value-of select="$tab"/>
		<xsl:value-of select="concat('typedef SubID2Type&lt;',@valueType,',VOCAB',string-length(@subID),'(')"/>
		<xsl:call-template name="vocab-subid">
			<xsl:with-param name="subid" select="@subID"/>
		</xsl:call-template>
		<xsl:value-of select="concat(')&gt; ',@subID,'ID')"/>
		<xsl:value-of select="$seminew"/>
	</xsl:template>
	<!--definition templates for functions-->
	<xsl:template match="n1:Struct" mode="definition-constructor">
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@className"/>
			<xsl:with-param name="classname" select="@className"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline)"/>
		<xsl:apply-templates select="n1:Variable|n1:Vector" mode="constructor-line">
		</xsl:apply-templates>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Wrapper" mode="definition-constructor">
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@className"/>
			<xsl:with-param name="classname" select="@className"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline,$newline)"/>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Variable" mode="definition-get">
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@name"/>
			<xsl:with-param name="returntype" select="@type"/>
			<xsl:with-param name="classname" select="../@className"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline)"/>
		<xsl:apply-templates select="." mode="get-body"/>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>	
	<xsl:template match="n1:Vector" mode="definition-get">
		<xsl:variable name="vector">
			<xsl:call-template name="vectorbottle-type">
			<xsl:with-param name="type" select="@valueType"/>
		</xsl:call-template>
		</xsl:variable>
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@name"/>
			<xsl:with-param name="returntype" select="$vector"/>
			<xsl:with-param name="classname" select="../@className"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline)"/>
		<xsl:apply-templates select="." mode="get-body"/>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>	
	<xsl:template match="n1:Variable" mode="definition-set">
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="concat('set',@name)"/>
			<xsl:with-param name="returntype" select="../@className"/>
			<xsl:with-param name="argtype" select="@type"/>
			<xsl:with-param name="varname" select="'v'"/>
			<xsl:with-param name="classname" select="../@className"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline)"/>
		<xsl:apply-templates select="." mode="set-body"/>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Bottle" mode="definition-get">
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@subID"/>
			<xsl:with-param name="returntype" select="@valueType"/>
			<xsl:with-param name="classname" select="../@className"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline)"/>
		<xsl:value-of select="concat($tab,'return get(',@subID,'ID());',$newline)"/>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>
	<xsl:template match="n1:Bottle" mode="definition-set">
		<xsl:text>inline </xsl:text>
		<xsl:call-template name="functioncall">
			<xsl:with-param name="name" select="@subID"/>
			<xsl:with-param name="returntype" select="'void'"/>
			<xsl:with-param name="argtype" select="@valueType"/>
			<xsl:with-param name="classname" select="../@className"/>
			<xsl:with-param name="varname" select="'v'"/>
		</xsl:call-template>
		<xsl:value-of select="concat(' {',$newline)"/>
		<xsl:value-of select="concat($tab,'set(v,',@subID,'ID());',$newline)"/>
		<xsl:value-of select="concat('}',$newline,$newline)"/>
	</xsl:template>
	<!--templates to add a line to the constructor of a StructBottle, one for each variable or vector-->
	<xsl:template match="n1:Variable" mode="constructor-line">
		<xsl:choose>
			<xsl:when test="contains($builtintypes,@type)">
			<xsl:variable name="fn">
				<xsl:call-template name="func-append">
					<xsl:with-param name="type" select="@type"/>
				</xsl:call-template>
				</xsl:variable>
				<xsl:variable name="dv">
					<xsl:call-template name="get-default-val">
						<xsl:with-param name="type" select="@type"/>
					</xsl:call-template>
				</xsl:variable>
				<xsl:value-of select="concat($tab,'add',$fn,'(&quot;',@name,'&quot;,',$dv,')',$seminew)"/>
			</xsl:when>
			<xsl:otherwise>
				<xsl:value-of select="concat($tab,'addlist(&quot;',@name,'&quot;); ')"/>
				<xsl:value-of select="concat(@name,'() = ',@type,'();',$newline)"/>
			</xsl:otherwise>
		</xsl:choose>
	</xsl:template>
	<xsl:template match="n1:Variable|n1:Vector" mode="get-body">
		<xsl:variable name="lt">
			<xsl:choose>
				<xsl:when test="@valueType">
					<xsl:call-template name="vectorbottle-type">
						<xsl:with-param name="type" select="@valueType"/>
					</xsl:call-template>
				</xsl:when>
				<xsl:otherwise>
					<xsl:value-of select="@type"/>
				</xsl:otherwise>
			</xsl:choose>
		</xsl:variable>
		<xsl:variable name="count" select="count(preceding-sibling::n1:Variable)+count(preceding-sibling::n1:Vector)"/>
		<xsl:variable name="fn">
			<xsl:call-template name="func-append">
				<xsl:with-param name="type" select="@type"/>
			</xsl:call-template>
		</xsl:variable>
		<xsl:choose>
			<xsl:when test="contains($builtintypes,$lt)">
				<xsl:value-of select="concat($tab,'return get',$fn,'(',2*$count+1,')',$seminew)"/>
			</xsl:when>
			<xsl:otherwise>
				<xsl:value-of select="concat($tab,'return static_cast&lt;',$lt,'&amp;&gt;(getlist(',2*$count+1,'))',$seminew)"/>
			</xsl:otherwise>
		</xsl:choose>
	</xsl:template>
	<xsl:template match="n1:Variable" mode="set-body">
		<xsl:variable name="count" select="count(preceding-sibling::n1:Variable)+count(preceding-sibling::n1:Vector)"/>
		<xsl:variable name="fn">
			<xsl:call-template name="func-append">
				<xsl:with-param name="type" select="@type"/>
			</xsl:call-template>
		</xsl:variable>
		<xsl:value-of select="concat($tab,'set',$fn,'(',2*$count+1,',v)',$seminew)"/>
		<xsl:value-of select="concat($tab,'return *this',$seminew)"/>
	</xsl:template>
	<xsl:template match="n1:Vector" mode="constructor-line">
		<xsl:value-of select="concat($tab,'addlist(&quot;',@name,'&quot;); ')"/>
		<xsl:value-of select="concat(@name,'() = ')"/>
		<xsl:call-template name="vectorbottle-type">
			<xsl:with-param name="type" select="@valueType"/>
		</xsl:call-template>
		<xsl:value-of select="concat('()',$seminew)"/>
	</xsl:template>
	<!--templates to create a function call with return type, function name and argument type-->
	<xsl:template name="functioncall">
		<xsl:param name="name"/>
		<xsl:param name="argtype" select="''"/>
		<xsl:param name="returntype" select="''"/>
		<xsl:param name="varname" select="''"/>
		<xsl:param name="classname" select="''"/>
		<xsl:choose>
			<xsl:when test="not($returntype)">
				<xsl:value-of select="''"/>
			</xsl:when>
			<xsl:when test="contains('void int double',$returntype)">
				<xsl:value-of select="concat($returntype,' ')"/>
			</xsl:when>
			<xsl:when test="contains('string',$returntype)">
				<xsl:value-of select="'yarp::os::ConstString '"/>
			</xsl:when>
			<xsl:when test="contains('vocab',$returntype)">
				<xsl:value-of select="'int '"/>
			</xsl:when>
			<xsl:otherwise>
				<xsl:value-of select="concat($returntype,'&amp; ')"/>
			</xsl:otherwise>
		</xsl:choose>
		<xsl:if test="$classname">
			<xsl:value-of select="concat($classname,'::')"/>
		</xsl:if>
		<xsl:value-of select="concat($name,'(')"/>
		<xsl:choose>
			<xsl:when test="not($argtype)">
				<xsl:value-of select="''"/>
			</xsl:when>
			<xsl:when test="contains('int double',$argtype)">
				<xsl:value-of select="$argtype"/>
			</xsl:when>
			<xsl:when test="contains('string',$argtype)">
				<xsl:value-of select="'const yarp::os::ConstString&amp;'"/>
			</xsl:when>
			<xsl:when test="contains('vocab',$argtype)">
				<xsl:value-of select="'int'"/>
			</xsl:when>
			<xsl:otherwise>
				<xsl:value-of select="concat('const ',$argtype,'&amp;')"/>
			</xsl:otherwise>
		</xsl:choose>
		<xsl:if test="$varname">
			<xsl:value-of select="concat(' ',$varname)"/>
		</xsl:if>
		<xsl:value-of select="')'"/>
	</xsl:template>
	<xsl:template name="vectorbottle-type">
		<xsl:param name="type"/>
		<xsl:choose>
			<xsl:when test="$type = 'int'">
				<xsl:value-of select="'IntVector'"/>
			</xsl:when>
			<xsl:when test="$type = 'double'">
				<xsl:value-of select="'DoubleVector'"/>
			</xsl:when>
			<xsl:when test="$type = 'string'">
				<xsl:value-of select="'StringVector'"/>
			</xsl:when>
			<xsl:when test="$type = 'vocab'">
				<xsl:value-of select="'VocabVector'"/>
			</xsl:when>
			<xsl:otherwise>
				<xsl:value-of select="concat('VectorBottle&lt;',$type,'&gt;')"/>
			</xsl:otherwise>
		</xsl:choose>
	</xsl:template>
	<!--recursive template to tokenize a subID for the yarp::os::VOCAB macros-->
	<xsl:template name="vocab-subid">
		<xsl:param name="subid"/>
		<xsl:variable name="letter" select="substring($subid,1,1)"/>
		<xsl:text>'</xsl:text><xsl:value-of select="$letter"/><xsl:text>'</xsl:text>
		<xsl:if test="string-length($subid) &gt; 1">
			<xsl:value-of select="','"/>
			<xsl:call-template name="vocab-subid">
				<xsl:with-param name="subid" select="substring($subid,2)"/>
			</xsl:call-template>
		</xsl:if>
	</xsl:template>
	<xsl:template name="comment-implementations">
		<xsl:param name="className"/>
		<xsl:value-of select="concat('/*========================================',$newline)"/>
		<xsl:value-of select="concat('* ',$className,' member function implementations',$newline)"/>
		<xsl:value-of select="concat('*=======================================*/',$newline,$newline)"/>
	</xsl:template>
	<!--some variable used throughout the document-->
	<xsl:template name="func-append">
		<xsl:param name="type"/>
		<xsl:choose>
			<xsl:when test="contains($builtintypes,$type)">
				<xsl:value-of select="$type"/>
			</xsl:when>
			<xsl:otherwise>
				<xsl:value-of select="'list'"/>
			</xsl:otherwise>
		</xsl:choose>
	</xsl:template>
	<xsl:template name="get-default-val">
		<xsl:param name="type"/>
		<xsl:choose>
			<xsl:when test="$type = 'int'">
				<xsl:value-of select="'0'"/>
			</xsl:when>
			<xsl:when test="$type = 'double'">
				<xsl:value-of select="'0.0'"/>
			</xsl:when>
			<xsl:when test="$type = 'string'">
				<xsl:text>&quot;&quot;</xsl:text>
			</xsl:when>
			<xsl:when test="$type = 'vocab'">
				<xsl:value-of select="'0'"/>
			</xsl:when>
		</xsl:choose>
	</xsl:template>
	<xsl:variable name="builtintypes">
		<xsl:value-of select="'int double string vocab'"/>
	</xsl:variable>
	<xsl:variable name="stringtype">
		<xsl:value-of select="'yarp::os::ConstString'"/>
	</xsl:variable>
	<xsl:variable name="vocabtype">
		<xsl:value-of select="int"/>
	</xsl:variable>
	<xsl:variable name="seminew">
		<xsl:value-of select="concat(';',$newline)"/>
	</xsl:variable>
	<xsl:variable name="newline">
		<xsl:text>&#10;</xsl:text>
	</xsl:variable>
	<xsl:variable name="tab">
		<xsl:text>&#09;</xsl:text>
	</xsl:variable>
</xsl:transform>
