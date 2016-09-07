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

#include "Arbiter.h"
#include "Body.h"
#include "World.h"

#include "glut.h"

Arbiter::Arbiter(Body* b1, Body* b2)
{
	if (b1 < b2)
	{
		body1 = b1;
		body2 = b2;
	}
	else
	{
		body1 = b2;
		body2 = b1;
	}

	numContacts = Collide(contacts, body1, body2);

	// 摩擦模型有很多麻烦的地方
	// 毕竟刚体碰撞的碰撞力是无限大, 对应的最大静摩擦力也就变成了无限大
	// 所以, 反正也算不对......
	// 见过的摩擦系数计算方案:
	// - 两个相比, 取大的
	// - 均方根
	// - 勾股定理
	// - ↓
	friction = sqrtf(body1->friction * body2->friction); // 不过这里是摩擦力还是阻尼呢

	// glPointSize(4.0f);
	// glColor3f(1.0f, 0.0f, 0.0f);
	// glBegin(GL_POINTS);
	// for (int i = 0; i < numContacts; ++i)
	// {
	// 	glVertex2f(contacts[i].position.x, contacts[i].position.y);
	// }
	// glEnd();
	// glPointSize(1.0f);
}

// 更新一个value怎么会这么长
// 以及
// 为什么不直接每次清空
void Arbiter::Update(Contact* newContacts, int numNewContacts)
{
	Contact mergedContacts[2];

	for (int i = 0; i < numNewContacts; ++i)
	{
		Contact* cNew = newContacts + i; // C++是这么取数组索引的, newContacts是数组首地址
		int k = -1;
		for (int j = 0; j < numContacts; ++j)
		{
			Contact* cOld = contacts + j; // 直接取索引
			if (cNew->feature.value == cOld->feature.value)
			{
				k = j;
				break;
			}
		}

		if (k > -1)
		{
			Contact* c = mergedContacts + i;
			Contact* cOld = contacts + k;
			*c = *cNew;
			if (World::warmStarting)
			{
				c->Pn = cOld->Pn;
				c->Pt = cOld->Pt;
				c->Pnb = cOld->Pnb;
			}
			else
			{
				c->Pn = 0.0f;
				c->Pt = 0.0f;
				c->Pnb = 0.0f;
			}
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}

	for (int i = 0; i < numNewContacts; ++i)
		contacts[i] = mergedContacts[i];

	numContacts = numNewContacts; // C++的作用域呀....
}

// 重要的地方开始了
void Arbiter::PreStep(float inv_dt)
{
	const float k_allowedPenetration = 0.01f; // 允许渗透值, 用来减小抖动?
	float k_biasFactor = World::positionCorrection ? 0.2f : 0.0f; // 加上额外的间隙吗?

	for (int i = 0; i < numContacts; ++i)
	{
		Contact* c = contacts + i;

		Vec2 r1 = c->position - body1->position;
		Vec2 r2 = c->position - body2->position;

		// Precompute normal mass, tangent mass, and bias.
		float rn1 = Dot(r1, c->normal); // r1在法线上的投影
		float rn2 = Dot(r2, c->normal);
		float kNormal = body1->invMass + body2->invMass;
		kNormal += body1->invI * (Dot(r1, r1) - rn1 * rn1) + body2->invI * (Dot(r2, r2) - rn2 * rn2);
		c->massNormal = 1.0f / kNormal;

		Vec2 tangent = Cross(c->normal, 1.0f); // 获取法线的垂线(碰撞切线)
		float rt1 = Dot(r1, tangent);
		float rt2 = Dot(r2, tangent);
		float kTangent = body1->invMass + body2->invMass;
		kTangent += body1->invI * (Dot(r1, r1) - rt1 * rt1) + body2->invI * (Dot(r2, r2) - rt2 * rt2);
		c->massTangent = 1.0f /  kTangent;

		c->bias = -k_biasFactor * inv_dt * Min(0.0f, c->separation + k_allowedPenetration);

		if (World::accumulateImpulses)
		{
			// Apply normal + friction impulse
			Vec2 P = c->Pn * c->normal + c->Pt * tangent;

			body1->velocity -= body1->invMass * P;
			body1->angularVelocity -= body1->invI * Cross(r1, P);

			body2->velocity += body2->invMass * P;
			body2->angularVelocity += body2->invI * Cross(r2, P);
		}
	}
}

void Arbiter::ApplyImpulse()
{
	Body* b1 = body1;
	Body* b2 = body2;

	for (int i = 0; i < numContacts; ++i)
	{
		Contact* c = contacts + i;
		c->r1 = c->position - b1->position;
		c->r2 = c->position - b2->position;

		// Relative velocity at contact
		Vec2 dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

		// Compute normal impulse
		float vn = Dot(dv, c->normal);

		float dPn = c->massNormal * (-vn + c->bias);

		if (World::accumulateImpulses)
		{
			// Clamp the accumulated impulse
			float Pn0 = c->Pn;
			c->Pn = Max(Pn0 + dPn, 0.0f);
			dPn = c->Pn - Pn0;
		}
		else
		{
			dPn = Max(dPn, 0.0f);
		}

		// Apply contact impulse
		Vec2 Pn = dPn * c->normal;

		b1->velocity -= b1->invMass * Pn;
		b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);

		b2->velocity += b2->invMass * Pn;
		b2->angularVelocity += b2->invI * Cross(c->r2, Pn);

		// Relative velocity at contact
		dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

		Vec2 tangent = Cross(c->normal, 1.0f);
		float vt = Dot(dv, tangent);
		float dPt = c->massTangent * (-vt);

		if (World::accumulateImpulses)
		{
			// Compute friction impulse
			float maxPt = friction * c->Pn;

			// Clamp friction
			float oldTangentImpulse = c->Pt;
			c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			dPt = c->Pt - oldTangentImpulse;
		}
		else
		{
			float maxPt = friction * dPn;
			dPt = Clamp(dPt, -maxPt, maxPt);
		}

		// Apply contact impulse
		Vec2 Pt = dPt * tangent;

		b1->velocity -= b1->invMass * Pt;
		b1->angularVelocity -= b1->invI * Cross(c->r1, Pt);

		b2->velocity += b2->invMass * Pt;
		b2->angularVelocity += b2->invI * Cross(c->r2, Pt);
	}
}
