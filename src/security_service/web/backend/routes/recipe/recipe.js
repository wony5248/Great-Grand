const express = require("express");
const app = express.Router();
const axios = require("axios");
const crypto = require('crypto');
const nodemailer = require('nodemailer');

require('dotenv').config();

const { pool } = require(`./../../mysql`);

app.get("/detail", async(req, res)=>{
    const rID = req.query.rID;
    let datas = [];
    try{
        const [rows1, fields1] = await pool.query("SELECT recipeName, recipeImage, recipeIntroduce, recipeAmount, recipeTime FROM Recipe WHERE rID = ?;", [
            rID
        ]);
        datas.push(rows1);

        const [rows2, fields2] = await pool.query("SELECT i.ingredientName, ri.ingredientAmount FROM Ingredient AS i JOIN RecipeIngredient AS ri ON i.iID=ri.iID WHERE ri.rID = ?;", [
            rID
        ]);
        datas.push(rows2);

        const [rows3, fields3] = await pool.query("SELECT recipephaseImage, recipephaseIntroduce FROM RecipePhase WHERE rID = ?;", [
            rID
        ]);
        datas.push(rows3);

        res.send(datas);
    }
    catch(e){
        console.log(e);
    }
})

app.get("/tenRecentRecipe", async(req, res) => {
    try {
        const [rows, fields] = await pool.query("SELECT rID, recipeName AS rName, recipeImage AS rImage FROM Recipe ORDER BY rID DESC LIMIT 10;");

        res.json(rows);
    }
    catch (e) {
        console.log('===========최근 레시피 조회 중 에러 발생===========');
        console.log(e);
    }
})

app.get("/tenFavorRecipe", async(req, res) => {
    try {
        const [rows, fields] = await pool.query("SELECT r.rID, r.recipeName AS rName, r.recipeImage AS rImage, count(*) FROM Favorites AS f JOIN Recipe AS r ON f.rID = r.rID GROUP BY r.rID ORDER BY count(*) DESC LIMIT 10;");
        
        res.json(rows);
    }
    catch (e) {
        console.log('===========즐겨찾기 탑텐 레시피 조회 중 에러 발생===========');
        console.log(e);
    }
})

app.get("/favorRecipe", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uid;

    try {
        const [rows1, fields1] = await pool.query("SELECT r.rID, r.recipeName AS rName, r.recipeIntroduce AS rIntroduce, r.recipeImage AS rImage FROM Favorites AS f JOIN Recipe AS r ON r.rID = f.rID WHERE f.uID = ?", [
            uID
        ]);

        res.json(rows1)
    }
    catch (err) {
        console.log('===========즐겨찾기 레시피 조회 중 에러 발생===========');
        console.log(err);
    }
})

app.get("/checkFavorRecipe", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uid;
    
    const rID = req.query.rID;

    try {
        const [rows1, fields1] = await pool.query("SELECT rID FROM Favorites WHERE uID = ? AND rID = ?", [
            uID,
            rID,
        ]);
        console.log("rows1", rows1);
        if(rows1.length === 0){
            res.send(false);
        }
        else{
            res.send(true);
        }
    }
    catch (err) {
        console.log('===========즐겨찾기 레시피 조회 중 에러 발생===========');
        console.log(err);
    }
})

app.post("/addFavorRecipe", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uid;
    
    const rID = req.body.rID;
    const isStar = req.body.isStar;

    console.log("uID", uID)
    console.log("rID", rID)
    console.log("isStar", isStar)

    try {
        let sql;
        if (isStar === true){
            sql = "DELETE FROM Favorites WHERE uID = ? AND rID = ?"
            const [rows1, fields1] = await pool.query(sql, [
                uID,
                rID,
            ]);
            res.send(false)
        }
        else{
            sql = "INSERT INTO Favorites VALUES (?, ?)"
            const [rows1, fields1] = await pool.query(sql, [
                uID,
                rID,
            ]);
            res.send(true)
        }
       
    }
    catch (err) {
        console.log('===========즐겨찾기 추가 중 에러 발생===========');
        console.log(err);
    }
})

app.post("/search", async(req, res) => {

    const cl2 = req.body.cl2;
    const len = cl2.length;

    let sql;

    if(len === 0){
        sql = "SELECT rID, recipeName, recipeImage, recipeTime FROM Recipe"
    }
    else{
        let cl2Str = "(";
        for (let i = 0; i < len; i++) {
            cl2Str = cl2Str + cl2[i];
            if (i === len - 1) {
                cl2Str = cl2Str + ")";
            }
            else {
                cl2Str = cl2Str + ",";
            }
        }

        sql = "SELECT r.rID, r.recipeName, r.recipeImage, r.recipeTime, rid.count \
        FROM Recipe r, (SELECT DISTINCT ri.rID, count(*) count FROM RecipeIngredient ri, Ingredient i \
        WHERE ri.iID = i.iID and i.ingredientName REGEXP(SELECT REPLACE(GROUP_CONCAT(a.classification2Name), ',', '|') AS NAME FROM(SELECT c2.classification2Name FROM Classification2 c2 WHERE c2.c2ID in " + cl2Str 
        + ") a) Group by ri.rID) rid WHERE r.rID = rid.rID Order by rid.count DESC;"
    }
    
    let list = [];

    try {
        const [rows1, fields1] = await pool.query(sql, []);

        // res.json(rows1) //rID들이 들어있음
        list.push(rows1);
        // res.json(rows1);

        let selIngredient=[];
        let cl2Name = [];  
        for (let i = 0; i < len; i++) { //소분류 아이디로 이름 가져오기
            const [rows3, fields3] = await pool.query("SELECT classification2Name FROM Classification2 WHERE c2ID = ?", [
                cl2[i]
            ]);
            cl2Name.push(rows3[0].classification2Name)
        }

        if (len !== 0) {
            const rows1_len = rows1.length;
            for (let i = 0; i < rows1_len; i++) {//레시피마다 반복

                //레시피마다 인그리디언트 이름들을 가져옴
                sql = "SELECT i.ingredientName \
                FROM Ingredient AS i JOIN RecipeIngredient AS ri \
                ON i.iID = ri.iID \
                WHERE ri.rID = ?"
                const [rows2, fields2] = await pool.query(sql, [
                    rows1[i].rID
                ]);
                // console.log("인그리디언트 이름들",rows2);

                let temp = [];
                const rows2_len = rows2.length;
                let sqlarr = "(";
                for (let j = 0; j < rows2_len; j++) { //인그리디언트 이름
                    for (let k = 0; k < len; k++) { //소분류 이름
                        let check = rows2[j].ingredientName.indexOf(cl2Name[k]);
                        //인그리디언트안에 소분류 이름이 포함되어있음!!
                        // console.log("재료 이름", rows2[j].ingredientName);
                        // console.log("소분류 이름", cl2Name[k]);
                        // console.log("check",check);
                        if (check !== -1) {
                            temp.push(rows2[j].ingredientName)
                            sqlarr += "'" + rows2[j].ingredientName + "'" + ","

                        }
                    }
                }
                sqlarr = sqlarr.slice(0, -1);
                sqlarr += ")";
                sql = "SELECT i.ingredientName \
                FROM Ingredient AS i JOIN RecipeIngredient AS ri \
                ON i.iID = ri.iID \
                WHERE ri.rID = ? AND i.ingredientName NOT IN " + sqlarr + " \
                LIMIT ?"
                const [rows4, fields4] = await pool.query(sql, [
                    rows1[i].rID,
                    6 - rows1[i].count,
                ]);

                const rows4_len = rows4.length;
                for (let j = 0; j < rows4_len; j++) {
                    temp.push(rows4[j].ingredientName);
                }
                selIngredient.push(temp);
            }
            list.push(selIngredient);
            res.json(list);
        }
        else {
            const rows1_len = rows1.length;
            for (let i = 0; i < rows1_len; i++){
                let temp = [];
                sql = "SELECT i.ingredientName \
                FROM Ingredient AS i JOIN RecipeIngredient AS ri \
                ON i.iID = ri.iID \
                WHERE ri.rID = ? \
                LIMIT 6"
                const [rows4, fields4] = await pool.query(sql, [
                    rows1[i].rID,
                ]);
                
                const rows4_len = rows4.length;
                for (let j = 0; j < rows4_len; j++) {
                    temp.push(rows4[j].ingredientName);
                }
                selIngredient.push(temp);
            }
            list.push(selIngredient);
            res.json(list);
        }
        

    }
    catch (err) {
        console.log('===========소분류로 레시피 조회 중 에러 발생===========');
        console.log(err);
    }
})

module.exports = app;
