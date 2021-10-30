// Fill out your copyright notice in the Description page of Project Settings.


#include "Turbulence/TurbulenceModel.h"

#define M_TO_FT (3.28084f)

UTurbulenceModel::UTurbulenceModel()
{
}

float UTurbulenceModel::MToFt(float M) const
{
	return M * M_TO_FT;
}

float UTurbulenceModel::FtToM(float Ft) const
{
	return Ft / M_TO_FT;
}