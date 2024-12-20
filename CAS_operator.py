from flask import Flask, redirect, url_for, render_template, session
from flask import request
from datetime import timedelta
from flask import flash
from flask_sqlalchemy import SQLAlchemy
import logging

logging.basicConfig(level=logging.DEBUG, format=' %(asctime)s - %(levelname)s - %(message)s')
logging.debug('custom Start of program')


app=Flask(__name__)
app.secret_key="hello"
app.config['SQLALCHEMY_DATABASE_URI']='sqlite:///users.sqlite3'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS']=False
# app.permanent_session_lifetime=timedelta(minutes=5)


db=SQLAlchemy(app)

class users(db.Model):
    _id=db.Column("id", db.Integer, primary_key=True)
    name=db.Column(db.String(100))
    email=db.Column(db.String(100))

    def __init__(self, name, email):
        self.name=name
        self.email=email


@app.route("/") # decorator to give the route
def home():
    return render_template("index.html")


@app.route("/view") # decorator to give the route
def view():
    return render_template("view.html", values=users.query.all())

@app.route("/login",methods=["POST","GET"])
def login():
    if request.method=="POST":
        # session.permanent=True
        user=request.form["nm"]
        session["user"]=user
        # print(session)
        found_user=users.query.filter_by(name=user).first()
        if found_user:
            session["email"]=found_user.email
            
        else:
            usr=users(user,"")
            db.session.add(usr)
            db.session.commit()
        flash("You have been logged in!", "info")
        logging.debug(f" Custom-> type of session is {type(session)}")
        logging.debug(f" Custom-> session is \n{session}") #<SecureCookieSession {'key': 'value'}>
        return redirect(url_for("user"))
    else:
        if "user" in session:
            flash("Already logged in!", "info")
            return redirect(url_for("user")) # redirect for is the name of the data
        return render_template("login.html")

@app.route("/user",methods=["POST", "GET"])
def user():
    email=None
    if "user" in session:
        user=session["user"]

        if request.method=="POST":
            email=request.form["email"] # this is the name of the input field
            session["email"]=email
            found_user=users.query.filter_by(name=user).first()
            found_user.email=email
            db.session.commit()
            flash("email was saved")
            logging.debug(f" Custom-> {session}")
        else:
            if "email" in session:
                email=session["email"]
        flash("hello user", "info")
        return render_template("user.html",email=email)
    else:
        flash("You are not logged in!", "info")
        return redirect(url_for("login"))


@app.route("/logout")
def logout():
    session.pop("user", None)
    session.pop("email",None)
    flash("You have been logged out!", "info")
    logging.debug("custom flash")
    return redirect(url_for("login"))


if __name__=="__main__":
    with app.app_context():
        db.create_all()
    app.run(debug=True)
