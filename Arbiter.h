/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef ARBITER_H
#define ARBITER_H

#include "MathUtils.h"

struct Body;

// 特征对
union FeaturePair
{
	struct Edges
	{
		char inEdge1;
		char outEdge1;
		char inEdge2;
		char outEdge2;
	} e; // ???
	int value;
};

struct Contact
{
	Contact() : Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}

	Vec2 position; // ? 谁的, 看起来像是只算一个点的?
	Vec2 normal;
	Vec2 r1, r2; // 碰撞半径, 碰撞点到物体重心的距离
	float separation;
	float Pn;	// accumulated normal impulse 法线冲量? 平动冲量???
	float Pt;	// accumulated tangent impulse 切线冲量? 旋转冲量???
	float Pnb;	// accumulated normal impulse for position bias 法线位置偏移冲量? 是用来校准位置的吗
	float massNormal, massTangent; // 法质量, 切质量, 什么鬼?!
	float bias;
	FeaturePair feature;
};

struct ArbiterKey
{
	ArbiterKey(Body* b1, Body* b2)
	{
		if (b1 < b2) // 这个大小是怎么比的, 没有找到运算符重载
		{
			body1 = b1; body2 = b2;
		}
		else
		{
			body1 = b2; body2 = b1;
		}
	}

	Body* body1;
	Body* body2;
};

struct Arbiter
{
	enum {MAX_POINTS = 2}; // 流形最多两个点

	Arbiter(Body* b1, Body* b2);

	void Update(Contact* contacts, int numContacts);

	void PreStep(float inv_dt);
	void ApplyImpulse();

	Contact contacts[MAX_POINTS]; // 接触点 们
	int numContacts;

	Body* body1;
	Body* body2;

	// Combined friction
	float friction;
};

// This is used by std::set
inline bool operator < (const ArbiterKey& a1, const ArbiterKey& a2)
{
	if (a1.body1 < a2.body1)
		return true;

	if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
		return true;

	return false;
}

int Collide(Contact* contacts, Body* body1, Body* body2);

#endif
